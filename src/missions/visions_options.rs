use crate::missions::Target;

pub fn lookfor(target: &Target) -> bool {
    match target {
        Target::Gate => true,
        Target::Shark => true,
        _ => false,
    }
}
