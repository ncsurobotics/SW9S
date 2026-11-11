use crate::missions::Target;

pub fn lookfor(target: &Target) -> bool {
    match target {
        Target::Gate => true,
        Target::Shark => true,
        Target::SlalomWhite => true,
        Target::SlalomRed => true,
        _ => true,
    }
}
