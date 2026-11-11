use crate::missions::Target;

#[allow(dead_code)]
pub enum VisionResult {
    Found,
    Searching,
    Failed,
}

pub fn lookfor(target: &Target) -> VisionResult {
    match target {
        Target::Gate => VisionResult::Searching,
        Target::Shark => VisionResult::Searching,
        Target::SlalomWhite => VisionResult::Searching,
        Target::SlalomRed => VisionResult::Searching,
        // Swimming with no visual target is a valid open-water action.
        Target::None => VisionResult::Found,
        Target::Path => VisionResult::Searching,
    }
}
