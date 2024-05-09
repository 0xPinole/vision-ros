"""Camera configurations source."""

import cv2


class CamConfig:
    """Camera fixed settings."""

    width = 480
    height = 640

    coefficients = [
        [
            -0.19080626244096968,
            -1.3701676357322958,
            0.020816122710077146,
            -0.003820545745672551,
            12.686703256756056,
        ]
    ]
    matrix = [
        [763.292471708863, 0.0, 309.7537601952385],
        [0.0, 748.6775405048527, 190.81088574322078],
        [0.0, 0.0, 1.0],
    ]

    video_flags = [
        (cv2.CAP_PROP_AUTOFOCUS, 0)
    ]  # (setting, value)
