from typing import Type


def enum_to_str(enum_class: Type, value: int) -> str:
    """Converts a ROS2 enum value to its string name.

    Args:
        enum_class: The ROS2 message class containing the enum constants.
        value: The integer value of the enum.

    Returns:
        str: The name of the enum constant, or the value with "UNKNOWN NAME" if not found.
    """
    mapping = {}
    # Iterate over all attributes in the enum class
    for attr_name in dir(enum_class):
        # Consider only uppercase attributes (common convention for enums)
        if attr_name.isupper():
            attr_value = getattr(enum_class, attr_name)
            # Check if the attribute is an integer (enum values are typically int)
            if isinstance(attr_value, int):
                mapping[attr_value] = attr_name
    return mapping.get(value, f"{value} :UNKNOWN NAME")
