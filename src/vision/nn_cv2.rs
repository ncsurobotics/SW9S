use anyhow::Result;
use derive_getters::Getters;
use opencv::{
    core::{Rect2d, Scalar, Size, VecN, Vector, CV_32F},
    dnn::{blob_from_image, read_net_from_onnx, read_net_from_onnx_buffer, Net},
    prelude::{Mat, MatTraitConst, NetTrait, NetTraitConst},
};
use std::hash::Hash;
use std::{
    fmt::Debug,
    ops::{Deref, DerefMut},
    sync::Mutex,
};

#[derive(Debug, Clone, Getters, PartialEq)]
pub struct YoloDetection {
    class_id: i32,
    confidence: f64,
    bounding_box: Rect2d,
}

#[derive(Debug, Clone, Getters)]
pub struct YoloClass<T> {
    pub identifier: T,
    pub confidence: f64,
}

impl<T: PartialEq> PartialEq<T> for YoloClass<T> {
    fn eq(&self, other: &T) -> bool {
        self.identifier == *other
    }
}

impl<T: PartialEq> PartialEq for YoloClass<T> {
    fn eq(&self, other: &Self) -> bool {
        self.identifier == other.identifier
    }
}

impl<T: PartialEq> Eq for YoloClass<T> {}

impl<T: Hash> Hash for YoloClass<T> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.identifier.hash(state)
    }
}

impl<T> TryFrom<YoloDetection> for YoloClass<T>
where
    T: TryFrom<i32>,
    <T as TryFrom<i32>>::Error: std::error::Error + Send + Sync,
{
    type Error = T::Error;
    fn try_from(val: YoloDetection) -> Result<Self, Self::Error> {
        Ok(YoloClass {
            identifier: val.class_id.try_into()?,
            confidence: val.confidence,
        })
    }
}

pub trait VisionModel: Debug + Sync + Send + Clone {
    type PostProcessArgs;
    type ModelOutput;

    /// Forward pass the matrix through the model, skipping post-processing
    fn forward(&mut self, image: &Mat) -> Self::ModelOutput;
    /// Convert output from a model into detections
    fn post_process_args(&self) -> Self::PostProcessArgs;
    fn post_process(
        args: Self::PostProcessArgs,
        output: Self::ModelOutput,
        threshold: f64,
    ) -> Vec<YoloDetection>;

    /// Full input -> output processing
    fn detect_yolo_v5(&mut self, image: &Mat, threshold: f64) -> Vec<YoloDetection> {
        let model_output = self.forward(image);
        Self::post_process(self.post_process_args(), model_output, threshold)
    }
    fn size(&self) -> Size;
}

/* -------------------------------------------------- */
/* --------------- ONNX implementation -------------- */
/* -------------------------------------------------- */

/// Wrapper to let Rust know Net is Send and Sync.
///
/// We know it doesn't rely on thread-local state, but Rust assumes all
/// pointers are not Send or Sync by default.
#[derive(Debug, Clone)]
struct NetWrapper(pub Net);

impl Deref for NetWrapper {
    type Target = Net;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for NetWrapper {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

unsafe impl Send for NetWrapper {}
unsafe impl Sync for NetWrapper {}

/// ONNX vision model running via OpenCV
#[derive(Debug)]
pub struct OnnxModel {
    net: Mutex<NetWrapper>,
    //out_blob_names: Vec<String>,
    num_objects: usize,
    //output: Vec<usize>,
    //output_description: Vec<Rect2d>,
    model_size: Size,
    factor: f64,
}

impl OnnxModel {
    /// Creates model from in memory byte buffer
    ///
    /// # Arguments:
    /// * `model_bytes` - ONNX model in u8
    /// * `model_size` - input image square dimensions (e.g. 640 for 640x640)
    /// * `num_objects` - number of objects model can output
    ///
    /// # Examples:
    ///
    /// ```
    /// use opencv::core::Vector;
    /// use sw9s_lib::vision::nn_cv2::OnnxModel;
    ///
    /// OnnxModel::from_bytes(
    ///     &Vector::from_slice(include_bytes!("models/buoy_320.onnx")),
    ///     320,
    ///     4,
    /// )
    /// .unwrap();
    /// ```
    pub fn from_bytes(
        model_bytes: &Vector<u8>,
        model_size: i32,
        num_objects: usize,
    ) -> Result<Self> {
        let net = read_net_from_onnx_buffer(model_bytes)?;

        Ok(Self {
            net: Mutex::new(NetWrapper(net)),
            num_objects,
            model_size: Size::new(model_size, model_size),
            factor: Self::size_to_factor(model_size),
        })
    }

    /// Creates model from file (use a local path)
    ///
    /// # Arguments:
    /// * `model_name` - path to ONNX model (from working directory)
    /// * `model_size` - input image square dimensions (e.g. 640 for 640x640)
    /// * `num_objects` - number of objects model can output
    ///
    /// # Examples:
    /// ```
    /// use opencv::core::Vector;
    /// use sw9s_lib::vision::nn_cv2::OnnxModel;
    ///
    /// OnnxModel::from_file("src/vision/models/buoy_320.onnx", 320, 4).unwrap();
    /// ```
    pub fn from_file(model_name: &str, model_size: i32, num_objects: usize) -> Result<Self> {
        let net = read_net_from_onnx(model_name)?;

        Ok(Self {
            net: Mutex::new(NetWrapper(net)),
            num_objects,
            model_size: Size::new(model_size, model_size),
            factor: Self::size_to_factor(model_size),
        })
    }

    /// Calculates coordinate factor based on model size
    fn size_to_factor(model_size: i32) -> f64 {
        640.0 / model_size as f64
    }

    fn get_output_names(net: &Net) -> Vector<String> {
        let out_layers = net
            .get_unconnected_out_layers()
            .expect("Error getting unconnected out layers from model.");
        let layer_names = net
            .get_layer_names()
            .expect("Error getting layer names from model.");

        Vector::from_iter(
            out_layers
                .iter()
                .map(|layer_num| layer_names.get((layer_num - 1) as usize).unwrap())
                .to_owned()
                .collect::<Vec<_>>()
                .iter()
                .map(|s| s.as_str()),
        )
    }

    pub fn get_net(&mut self) -> &mut Net {
        self.net.get_mut().unwrap()
    }

    pub fn get_model_size(&self) -> Size {
        self.model_size
    }
}

impl Clone for OnnxModel {
    fn clone(&self) -> Self {
        Self {
            net: Mutex::new(self.net.lock().unwrap().clone()),
            num_objects: self.num_objects,
            model_size: self.model_size,
            factor: self.factor,
        }
    }
}

/// Loads model from file, mostly at compile time
///
/// # Arguments:
/// * `model_name` - path to ONNX model (relative to file function is called from)
/// * `model_size` - input image square dimensions (e.g. 640 for 640x640)
/// * `num_objects` - number of objects model can output
///
/// # Examples:
///
/// ```
/// use sw9s_lib::{
///     load_onnx,
///     vision::nn_cv2::OnnxModel,
/// };
///
/// let model: OnnxModel = load_onnx!("models/dummy_model.onnx", 320, 4);
/// ```
#[macro_export]
macro_rules! load_onnx {
    ($model_name:expr, $model_size:expr, $num_objects:expr) => {{
        use opencv::core::Vector;

        OnnxModel::from_bytes(
            &Vector::from_slice(include_bytes!("models/dummy_model.onnx")),
            $model_size,
            $num_objects,
        )
        .unwrap()
    }};
}

impl VisionModel for OnnxModel {
    fn detect_yolo_v5(&mut self, image: &Mat, threshold: f64) -> Vec<YoloDetection> {
        let result = self.forward(image);
        let post_processing = Self::process_net(self.num_objects, self.factor, result, threshold);

        post_processing
    }

    fn forward(&mut self, image: &Mat) -> Self::ModelOutput {
        let mut result: Vector<Mat> = Vector::new();
        let result_names = Self::get_output_names(&self.net.lock().unwrap());
        let blob = blob_from_image(
            image,
            1.0 / 255.0,
            self.model_size,
            Scalar::from(0.0),
            true,
            false,
            CV_32F,
        )
        .unwrap();

        self.net
            .lock()
            .unwrap()
            .set_input(&blob, "", 1.0, Scalar::from(0.0))
            .unwrap();
        self.net
            .lock()
            .unwrap()
            .forward(&mut result, &result_names)
            .unwrap();

        result
    }

    type ModelOutput = Vector<Mat>;

    type PostProcessArgs = (usize, f64);

    fn post_process_args(&self) -> Self::PostProcessArgs {
        (self.num_objects, self.factor)
    }

    fn post_process(
        args: Self::PostProcessArgs,
        output: Self::ModelOutput,
        threshold: f64,
    ) -> Vec<YoloDetection> {
        let post_processing = Self::process_net(args.0, args.1, output, threshold);

        post_processing
    }

    fn size(&self) -> Size {
        self.model_size
    }
}

impl OnnxModel {
    #[allow(unused)]
    /// Returns all detections from a net's output
    ///
    /// # Arguments
    /// * `result` - iterator of net output
    /// * `threshold` - minimum confidence
    fn process_net<I>(
        num_objects: usize,
        factor: f64,
        result: I,
        threshold: f64,
    ) -> Vec<YoloDetection>
    where
        I: IntoIterator<Item = Mat>,
    {
        result
            .into_iter()
            .flat_map(|level| -> Vec<YoloDetection> {
                // This reshape is always valid as per the model design
                let level = level
                    .reshape(1, (level.total() / (5 + num_objects)) as i32)
                    .unwrap();

                (0..level.rows())
                    .map(|idx| level.row(idx).unwrap())
                    .filter_map(|row| -> Option<YoloDetection> {
                        // Cols is always > 5. The column range can always be constructed, since
                        // row always has level.cols number of columns.
                        let scores = row
                            .col_range(&opencv::core::Range::new(5, level.cols()).unwrap())
                            .unwrap();

                        let mut max_loc = 5;
                        for idx in 6..level.cols() {
                            if row.at::<VecN<f32, 1>>(max_loc).unwrap()[0]
                                < row.at::<VecN<f32, 1>>(idx).unwrap()[0]
                            {
                                max_loc = idx;
                            }
                        }
                        max_loc -= 5;

                        // Always a valid index access
                        let confidence: f64 = row.at::<VecN<f32, 1>>(4).unwrap()[0].into();

                        if confidence > threshold {
                            // The given constant values are always valid indicies
                            let adjust_base = |idx: i32| -> f64 {
                                f64::from(row.at::<VecN<f32, 1>>(idx).unwrap()[0]) * factor
                            };

                            let x_adjust = |idx: i32| -> f64 { adjust_base(idx) / 640.0 * 800.0 };
                            let y_adjust = |idx: i32| -> f64 { adjust_base(idx) / 640.0 * 600.0 };

                            let (center_x, center_y, width, height) =
                                (x_adjust(0), y_adjust(1), x_adjust(2), y_adjust(3));

                            let left = center_x - width / 2.0;
                            let top = center_y - height / 2.0;

                            Some(YoloDetection {
                                class_id: max_loc,
                                confidence,
                                bounding_box: Rect2d {
                                    x: left,
                                    y: top,
                                    width,
                                    height,
                                },
                            })
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .collect()
    }
}
