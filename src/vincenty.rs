use crate::point::Point;
use std::f32;

#[allow(non_snake_case)]
pub fn vincenty_distance<T: Point>(fix1: &T, fix2: &T) -> f32 {
    let a = 6378137.;
    let b = 6356752.314245;
    let f = 1. / 298.257223563; // WGS-84 ellipsoid params

    // Difference in longitude

    let L = (fix2.longitude() - fix1.longitude()).to_radians();
    // Reduced latitude (latitude on the auxiliary sphere)
    let U1 = ((1_f32 - f) * fix1.latitude().to_radians().tan()).atan();
    // Reduced latitude (latitude on the auxiliary sphere)
    let U2 = ((1_f32 - f) * fix2.latitude().to_radians().tan()).atan();
    let (sinU1, cosU1) = U1.sin_cos();
    let (sinU2, cosU2) = U2.sin_cos();
    let mut cosSqAlpha;
    let mut sinSigma;
    let mut cos2SigmaM;
    let mut cosSigma;
    let mut sigma;
    // Longitude of the points on the auxiliary sphere
    let mut lambda = L;
    let mut lambdaP;
    let mut iterLimit = 100;

    loop {
        let (sinLambda, cosLambda) = lambda.sin_cos();
        sinSigma = ((cosU2 * sinLambda) * (cosU2 * sinLambda)
            + (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
                * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda))
            .sqrt();

        if sinSigma == 0_f32 {
            return 0_f32;
        }

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = sinSigma.atan2(cosSigma);
        let sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1_f32 - sinAlpha * sinAlpha;

        if cosSqAlpha == 0_f32 {
            // equatorial geodesics require special handling
            // per [Algorithms for geodesics, Charles F. F. Karney](https://arxiv.org/pdf/1109.4448.pdf)
            cos2SigmaM = 0_f32;
        } else {
            cos2SigmaM = cosSigma - 2_f32 * sinU1 * sinU2 / cosSqAlpha;
        }

        let C = f / 16_f32 * cosSqAlpha * (4_f32 + f * (4_f32 - 3_f32 * cosSqAlpha));
        lambdaP = lambda;
        lambda = L
            + (1_f32 - C)
                * f
                * sinAlpha
                * (sigma
                    + C * sinSigma
                        * (cos2SigmaM + C * cosSigma * (-1_f32 + 2_f32 * cos2SigmaM * cos2SigmaM)));

        // 10−12 corresponds to approximately 0.06 mm
        if (lambda - lambdaP).abs() <= 1e-6 {
            break;
        }

        iterLimit -= 1;

        if iterLimit == 0 {
            break;
        }
    }

    if iterLimit == 0 {
        return 0.0;
    }

    let uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    let A =
        1_f32 + uSq / 16384_f32 * (4096_f32 + uSq * (-768_f32 + uSq * (320_f32 - 175_f32 * uSq)));
    let B = uSq / 1024_f32 * (256_f32 + uSq * (-128_f32 + uSq * (74_f32 - 47_f32 * uSq)));

    let deltaSigma = B
        * sinSigma
        * (cos2SigmaM
            + B / 4_f32
                * (cosSigma * (-1_f32 + 2_f32 * cos2SigmaM * cos2SigmaM)
                    - B / 6_f32
                        * cos2SigmaM
                        * (-3_f32 + 4_f32 * sinSigma * sinSigma)
                        * (-3_f32 + 4_f32 * cos2SigmaM * cos2SigmaM)));

    let s = b * A * (sigma - deltaSigma);
    s / 1000_f32
}
