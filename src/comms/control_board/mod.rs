use core::fmt::Debug;
use std::{ops::Deref, sync::Arc, time::Duration};

use anyhow::{anyhow, bail, Result};
use thiserror::Error;
use tokio::{
    io::{self, AsyncRead, AsyncWrite, AsyncWriteExt, WriteHalf},
    net::TcpStream,
    sync::Mutex,
    time::{sleep, timeout},
};
use tokio_serial::{DataBits, Parity, SerialStream, StopBits};

use self::{
    response::ResponseMap,
    util::{Angles, BNO055AxisConfig},
};

use super::auv_control_board::{AUVControlBoard, MessageId};
use crate::logln;

pub mod response;
pub mod util;

pub enum SensorStatuses {
    ImuNr,
    DepthNr,
    AllGood,
}

pub static LAST_YAW: std::sync::Mutex<Option<f32>> = std::sync::Mutex::new(None);

#[derive(Debug)]
pub struct ControlBoard<T>
where
    T: AsyncWriteExt + Unpin,
{
    inner: Arc<AUVControlBoard<T, ResponseMap>>,
    initial_angles: Arc<Mutex<Option<Angles>>>,
}

impl<T: AsyncWriteExt + Unpin> Deref for ControlBoard<T> {
    type Target = AUVControlBoard<T, ResponseMap>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

type ThrusterInversions = Vec<bool>;
type DofSpeeds = [f32; 6];

pub struct VehicleDefinition {
    pub(crate) motor_matrix: MotorMatrix,
    pub(crate) thruster_inversions: ThrusterInversions,
    pub(crate) dof_speeds: DofSpeeds,
}

impl VehicleDefinition {
    pub fn new(
        motor_matrix: MotorMatrix,
        thruster_inversions: ThrusterInversions,
        dof_speeds: DofSpeeds,
    ) -> Result<Self, ControlBoardError> {
        use ControlBoardError::*;

        let rows = motor_matrix.0.iter().map(|r| r.is_some() as u8).sum();
        let inversions = thruster_inversions.len();

        if rows != inversions as u8 {
            Err(ThrusterMismatch { rows, inversions })
        } else {
            Ok(Self {
                motor_matrix,
                thruster_inversions,
                dof_speeds,
            })
        }
    }
}

/// Contains a *populated* motor matrix.
/// Can only be created using [`MotorMatrixBuilder`].
#[derive(Debug)]
pub struct MotorMatrix(pub(crate) [Option<MotorMatrixRowParams>; 8]);

impl MotorMatrix {
    /// Creates a new [`MotorMatrixBuilder`]
    pub fn builder(thrusters_in_use: u8) -> MotorMatrixBuilder {
        MotorMatrixBuilder::new(thrusters_in_use)
    }
}

/// A builder for MotorMatrix. This struct makes it cleaner to define thruster parameters,
/// as well as validate that the correct amount of thrusters have a definition.
#[derive(Debug)]
pub struct MotorMatrixBuilder {
    pub(crate) params: [Option<MotorMatrixRowParams>; 8],
    pub(crate) thrusters_in_use: u8,
}

impl MotorMatrixBuilder {
    /// Creates an unpopulated builder.
    pub fn new(thrusters_in_use: u8) -> Self {
        MotorMatrixBuilder {
            params: [None, None, None, None, None, None, None, None],
            thrusters_in_use,
        }
    }

    /// Sets the parameters for a specific thruster.
    pub fn set_row(mut self, thruster: u8, parameters: MotorMatrixRowParams) -> Self {
        let thruster_index = thruster - 1;
        self.params[thruster_index as usize] = Some(parameters);
        self
    }

    /// Build the populated [`MotorMatrix`]
    pub fn build(self) -> MotorMatrix {
        // Make sure that we currently have parameters for `thrusters_in_use` thrusters.
        let thrusters_defined = self.params.iter().filter(|x| x.is_some()).count();

        assert_eq!(thrusters_defined, self.thrusters_in_use as usize);

        // Make sure that all of the arrays contain `Some()`
        MotorMatrix(self.params)
    }
}

/// Each row corresponds to a line of self.motor_matrix_set(...);
/// Can either be created manually, or the parameters can be dumped into [`Self::new`]
/// for shorter code.
#[derive(Debug)]
pub struct MotorMatrixRowParams {
    x: f32,
    y: f32,
    z: f32,
    pitch: f32,
    roll: f32,
    yaw: f32,
}

impl MotorMatrixRowParams {
    pub fn new(x: f32, y: f32, z: f32, pitch: f32, roll: f32, yaw: f32) -> Self {
        Self {
            x,
            y,
            z,
            pitch,
            roll,
            yaw,
        }
    }
}

impl From<[f32; 6]> for MotorMatrixRowParams {
    fn from(vals: [f32; 6]) -> Self {
        Self {
            x: vals[0],
            y: vals[1],
            z: vals[2],
            pitch: vals[3],
            roll: vals[4],
            yaw: vals[5],
        }
    }
}

#[derive(Error, Debug)]
pub enum ControlBoardError {
    #[error("number of motor matrix rows and number of thruster inversions do not match: {rows:?} rows, {inversions:?} inversions")]
    ThrusterMismatch { rows: u8, inversions: usize },
}

impl<T: 'static + AsyncWriteExt + Unpin + Send> ControlBoard<T> {
    pub async fn new<U>(
        comm_out: T,
        comm_in: U,
        msg_id: Option<MessageId>,
        vehicle_defintion: VehicleDefinition,
    ) -> Result<Self>
    where
        U: 'static + AsyncRead + Unpin + Send,
    {
        let msg_id = msg_id.unwrap_or_default();
        let responses = ResponseMap::new(comm_in).await;
        let this = Self {
            inner: AUVControlBoard::new(Mutex::from(comm_out).into(), responses, msg_id).into(),
            initial_angles: Arc::default(),
        };

        // const THRUSTER_COUNT: u8 = 8;
        // let motor_matrix = MotorMatrixBuilder::new(THRUSTER_COUNT)
        //     .set_row(1, MotorMatrixRowParams::new(-1.0, 1.0, 0.0, 0.0, 0.0, -1.0))
        //     .set_row(2, MotorMatrixRowParams::new(1.0, 1.0, 0.0, 0.0, 0.0, 1.0))
        //     .set_row(3, MotorMatrixRowParams::new(-1.0, -1.0, 0.0, 0.0, 0.0, 1.0))
        //     .set_row(4, MotorMatrixRowParams::new(1.0, -1.0, 0.0, 0.0, 0.0, -1.0))
        //     .set_row(5, MotorMatrixRowParams::new(0.0, 0.0, -1.0, 1.0, -1.0, 0.0))
        //     .set_row(6, MotorMatrixRowParams::new(0.0, 0.0, -1.0, 1.0, 1.0, 0.0))
        //     .set_row(7, MotorMatrixRowParams::new(0.0, 0.0, -1.0, -1.0, -1.0, 0.0))
        //     .set_row(8, MotorMatrixRowParams::new(0.0, 0.0, -1.0, -1.0, 1.0, 0.0))
        //     .build();

        this.init_matrices(vehicle_defintion.motor_matrix).await?;
        this.thruster_inversion_set(vehicle_defintion.thruster_inversions)
            .await?;
        this.relative_dof_speed_set_batch(&vehicle_defintion.dof_speeds)
            .await?;
        this.bno055_imu_axis_config(BNO055AxisConfig::P6).await?;

        loop {
            if let Ok(ret) = timeout(Duration::from_secs(1), this.raw_speed_set([0.0; 8])).await {
                ret?;
                break;
            }
        }

        // Control board needs time to get its life together
        sleep(Duration::from_secs(5)).await;

        this.stab_tune().await?;

        let inner_clone = this.inner.clone();

        tokio::spawn(async move {
            loop {
                if (timeout(
                    Duration::from_millis(100),
                    Self::feed_watchdog(&inner_clone),
                )
                .await)
                    .is_err()
                {
                    logln!("Watchdog ACK timed out.");
                }

                sleep(Duration::from_millis(200)).await;
            }
        });

        // Wait for watchdog to register
        while this.watchdog_status().await != Some(true) {
            sleep(Duration::from_millis(10)).await;
        }
        Ok(this)
    }

    async fn init_matrices(&self, motor_matrix: MotorMatrix) -> Result<()> {
        for (i, _row) in motor_matrix.0.iter().enumerate() {
            // If the row is defined for the thruster, then set it
            if let Some(row) = _row {
                self.motor_matrix_set(i as u8, row.x, row.y, row.z, row.pitch, row.roll, row.yaw)
                    .await?;
            }
        }

        self.motor_matrix_update().await
    }

    async fn stab_tune(&self) -> Result<()> {
        self.stability_assist_pid_tune('X', 0.8, 0.0, 0.0, 0.6, false)
            .await?;
        self.stability_assist_pid_tune('Y', 2.0, 0.0, 0.0, 0.1, false)
            .await?;
        self.stability_assist_pid_tune('Z', 4.0, 0.0, 0.0, 1.0, false)
            .await?;
        self.stability_assist_pid_tune('D', 1.5, 0.0, 0.0, 1.0, false)
            .await
    }
}

impl ControlBoard<WriteHalf<SerialStream>> {
    pub async fn serial(port_name: &str, vehicle_defintion: VehicleDefinition) -> Result<Self> {
        const BAUD_RATE: u32 = 9600;
        const DATA_BITS: DataBits = DataBits::Eight;
        const PARITY: Parity = Parity::None;
        const STOP_BITS: StopBits = StopBits::One;

        let port_builder = tokio_serial::new(port_name, BAUD_RATE)
            .data_bits(DATA_BITS)
            .parity(PARITY)
            .stop_bits(STOP_BITS);
        let (comm_in, comm_out) = io::split(SerialStream::open(&port_builder)?);
        Self::new(comm_out, comm_in, None, vehicle_defintion).await
    }
}

impl ControlBoard<WriteHalf<TcpStream>> {
    /// Both connections are necessary for the simulator to run,
    /// but the one that doesn't feed forward to control board is unnecessary
    pub async fn tcp(
        host: &str,
        port: &str,
        dummy_port: String,
        vehicle_defintion: VehicleDefinition,
    ) -> Result<Self> {
        let host = host.to_string();
        let host_clone = host.clone();
        tokio::spawn(async move {
            let _stream = TcpStream::connect(host_clone + ":" + &dummy_port)
                .await
                .unwrap();
            // Have to avoid dropping the TCP stream
            loop {
                sleep(Duration::MAX).await
            }
        });

        let stream = TcpStream::connect(host.to_string() + ":" + port).await?;
        let (comm_in, comm_out) = io::split(stream);
        Self::new(comm_out, comm_in, None, vehicle_defintion).await
    }
}

impl<T: AsyncWrite + Unpin> ControlBoard<T> {
    pub async fn feed_watchdog(control_board: &Arc<AUVControlBoard<T, ResponseMap>>) -> Result<()> {
        const WATCHDOG_FEED: [u8; 4] = *b"WDGF";
        let message = Vec::from(WATCHDOG_FEED);
        control_board.write_out_basic(message).await
    }

    /// <https://mb3hel.github.io/AUVControlBoard/user_guide/messages/#configuration-commands>
    #[allow(clippy::too_many_arguments)]
    pub async fn motor_matrix_set(
        &self,
        thruster: u8,
        x: f32,
        y: f32,
        z: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
    ) -> Result<()> {
        const MOTOR_MATRIX_SET: [u8; 5] = *b"MMATS";
        // Oversized to avoid reallocations
        let mut message: Vec<u8> = Vec::with_capacity(32 * 8);
        message.extend(MOTOR_MATRIX_SET);

        if !(1..=8).contains(&thruster) {
            bail!("{thruster} is outside the allowed range 1-8.")
        };

        message.extend(thruster.to_le_bytes());
        [x, y, z, pitch, roll, yaw]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn motor_matrix_update(&self) -> Result<()> {
        const MOTOR_MATRIX_UPDATE: [u8; 5] = *b"MMATU";
        self.write_out_basic(Vec::from(MOTOR_MATRIX_UPDATE)).await
    }

    /// Set thruster inversions
    ///
    /// # Arguments:
    /// * `inversions` - Array of invert statuses, with motor 1 at index 0
    pub async fn thruster_inversion_set(&self, inversions: Vec<bool>) -> Result<()> {
        const THRUSTER_INVERSION_SET: [u8; 4] = *b"TINV";
        let mut message = Vec::from(THRUSTER_INVERSION_SET);

        // bitmask to u8, may or may not outpreform mutating a single u8
        message.push(
            inversions
                .iter()
                .enumerate()
                .map(|(idx, &inv)| (inv as u8) << idx)
                .sum(),
        );
        self.write_out_basic(message).await
    }

    pub async fn relative_dof_speed_set(
        &self,
        x: f32,
        y: f32,
        z: f32,
        xrot: f32,
        yrot: f32,
        zrot: f32,
    ) -> Result<()> {
        self.relative_dof_speed_set_batch(&[x, y, z, xrot, yrot, zrot])
            .await
    }

    pub async fn relative_dof_speed_set_batch(&self, values: &[f32; 6]) -> Result<()> {
        const DOF_SET: [u8; 6] = *b"RELDOF";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(DOF_SET);

        values
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn raw_speed_set(&self, speeds: [f32; 8]) -> Result<()> {
        const RAW_SET: [u8; 3] = *b"RAW";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(RAW_SET);

        speeds
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn global_speed_set(
        &self,
        x: f32,
        y: f32,
        z: f32,
        pitch_speed: f32,
        roll_speed: f32,
        yaw_speed: f32,
    ) -> Result<()> {
        const GLOBAL_SET: [u8; 6] = *b"GLOBAL";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(GLOBAL_SET);

        [x, y, z, pitch_speed, roll_speed, yaw_speed]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn stability_2_speed_set(
        &self,
        x: f32,
        y: f32,
        target_pitch: f32,
        target_roll: f32,
        target_yaw: f32,
        target_depth: f32,
    ) -> Result<()> {
        const SASSIST_2: [u8; 8] = *b"SASSIST2";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(SASSIST_2);

        [x, y, target_pitch, target_roll, target_yaw, target_depth]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        *LAST_YAW.lock().unwrap() = Some(target_yaw);
        self.write_out_basic(message).await
    }

    pub async fn set_initial_angle(&self) -> Result<()> {
        *self.initial_angles.lock().await = match self.responses().get_angles().await {
            Some(angle) => Some(angle),
            None => {
                self.bno055_periodic_read(true).await?;
                let mut angle = self.responses().get_angles().await;
                while angle.is_none() {
                    sleep(Duration::from_millis(50)).await;
                    angle = self.responses().get_angles().await;
                }
                angle
            }
        };
        Ok(())
    }

    pub async fn stability_2_speed_set_initial_yaw(
        &self,
        x: f32,
        y: f32,
        target_pitch: f32,
        target_roll: f32,
        target_depth: f32,
    ) -> Result<()> {
        const SASSIST_2: [u8; 8] = *b"SASSIST2";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(SASSIST_2);

        let self_angle = *self.initial_angles.lock().await;
        let target_yaw = match self_angle {
            Some(x) => *x.yaw(),
            None => {
                self.set_initial_angle().await?;
                let angle =
                    (*self.initial_angles.lock().await).ok_or(anyhow!("Initial Yaw set Error"))?;
                *angle.yaw()
            }
        };

        [x, y, target_pitch, target_roll, target_yaw, target_depth]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn stability_1_speed_set(
        &self,
        x: f32,
        y: f32,
        yaw_speed: f32,
        target_pitch: f32,
        target_roll: f32,
        target_depth: f32,
    ) -> Result<()> {
        const SASSIST_1: [u8; 8] = *b"SASSIST1";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(SASSIST_1);

        [x, y, yaw_speed, target_pitch, target_roll, target_depth]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));

        self.write_out_basic(message).await
    }

    pub async fn bno055_imu_axis_config(&self, config: BNO055AxisConfig) -> Result<()> {
        const BNO055A_CONFIG: [u8; 7] = *b"BNO055A";

        let mut message = Vec::from(BNO055A_CONFIG);
        message.push(config.into());

        self.write_out_basic(message).await
    }

    pub async fn bno055_periodic_read(&self, enable: bool) -> Result<()> {
        const BNO055P: [u8; 7] = *b"BNO055P";

        let mut message = Vec::from(BNO055P);
        message.push(enable.into());

        self.write_out_basic(message).await?;
        sleep(Duration::from_millis(300)).await; // Initialization time
        Ok(())
    }

    pub async fn stability_assist_pid_tune(
        &self,
        which: char,
        kp: f32,
        ki: f32,
        kd: f32,
        limit: f32,
        invert: bool,
    ) -> Result<()> {
        const STAB_TUNE: [u8; 9] = *b"SASSISTTN";
        // Oversized to avoid reallocations
        let mut message = Vec::with_capacity(32 * 8);
        message.extend(STAB_TUNE);

        if !['X', 'Y', 'Z', 'D'].contains(&which) {
            bail!("{which} is not a valid PID tune, pick from [X, Y, Z, D]")
        }
        message.push(which as u8);

        [kp, ki, kd, limit]
            .iter()
            .for_each(|val| message.extend(val.to_le_bytes()));
        message.push(invert as u8);

        self.write_out_basic(message).await
    }

    pub async fn sensor_status_query(&self) -> Result<SensorStatuses> {
        const STATUS: [u8; 5] = *b"SSTAT";
        let message = Vec::from(STATUS);
        let status_resp = self.write_out(message).await;
        let status_byte = status_resp.unwrap()[0];
        if status_byte & 0x10 != 0x10 {
            Ok(SensorStatuses::ImuNr)
        } else if status_byte & 0x01 != 0x01 {
            return Ok(SensorStatuses::DepthNr);
        } else {
            return Ok(SensorStatuses::AllGood);
        }
    }

    pub async fn reset(self) -> Result<()> {
        const RESET: [u8; 5] = *b"RESET";

        let mut message: Vec<_> = RESET.into();
        message.extend_from_slice(&[0x0D, 0x1E]);

        self.write_out_no_response(message).await?;
        sleep(Duration::from_secs(2)).await; // Reset time
        Ok(())
    }

    pub async fn watchdog_status(&self) -> Option<bool> {
        *self.responses().watchdog_status().read().await
    }

    pub async fn get_initial_angles(&self) -> Option<Angles> {
        *self.initial_angles.lock().await
    }
}
