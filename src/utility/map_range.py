def map_range(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return int(rightMin + (valueScaled * rightSpan))

def torque_to_PWM(value, motor):
    # Preset maps for Torque to PWM
    fromMin, fromMax, toMin, toMax = 0, 50, 1000, 2000
    # Snap input value to the PWM range
    if(value>fromMax):
        value = fromMax
    if(value<fromMin):
        value = fromMin
    # Figure out how 'wide' each range is
    fromSpan = fromMax - fromMin
    toSpan = toMax - toMin

    # Convert the from range into a 0-1 range (float)
    valueScaled = float(value - fromMin) / float(fromSpan)

    # Convert the 0-1 range into a value in the to range.
    val = int(toMin + (valueScaled * toSpan))
    return val

def map_pid_value(des):
    return map_range(des, 0, des, 1, 5)