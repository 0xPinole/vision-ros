"""Class owner of data format for objects."""
class Detection:
    x_1: int = None
    y_1: int = None
    x_2: int = None
    y_2: int = None
    aisle: int = None
    shelf: int = None
    code_name: str = None


class MechanismParameters:
    maximum_height: int = 360
    minimun_height: int = 360
