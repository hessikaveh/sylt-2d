use crate::arbiter::ArbiterErrors;
use crate::math_utils::MathErrors;
use std::fmt;

#[derive(Debug)]
pub enum Sylt2DErrors {
    MathOperations(MathErrors),
    Arbiter(ArbiterErrors),
}

impl fmt::Display for Sylt2DErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Sylt2DErrors::MathOperations(err) => write!(
                f,
                "In performing math operations the following error occured: {}",
                err
            ),
            Sylt2DErrors::Arbiter(err)=> write!(f, "In updating and finding the contacts between objects the following error occured: {}", err),
        }
    }
}

impl std::error::Error for Sylt2DErrors {}

impl From<MathErrors> for Sylt2DErrors {
    fn from(err: MathErrors) -> Self {
        Sylt2DErrors::MathOperations(err)
    }
}

impl From<ArbiterErrors> for Sylt2DErrors {
    fn from(value: ArbiterErrors) -> Self {
        Sylt2DErrors::Arbiter(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_utils::{Mat2x2, MathErrors, Vec2};
    #[test]
    fn test_invert_no_inverse() {
        // Test with a singular matrix (determinant == 0)
        let matrix = Mat2x2 {
            col1: Vec2::new(1.0, 2.0),
            col2: Vec2::new(2.0, 4.0),
        };
        fn invert(matrix: Mat2x2) -> Result<(), Sylt2DErrors> {
            matrix.invert()?;
            Ok(())
        }

        let inverted = invert(matrix);
        assert!(inverted.is_err());
        if let Err(ref err) = inverted {
            eprintln!("Error: {}", err);
        }
        if let Err(Sylt2DErrors::MathOperations(MathErrors::NoInverse { matrix: err_matrix })) =
            inverted
        {
            assert_eq!(err_matrix, matrix);
        } else {
            panic!("Expected NoInverse error");
        }
    }
}
