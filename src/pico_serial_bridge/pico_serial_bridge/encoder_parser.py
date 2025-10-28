def parse_encoder_data(line):
    """
    Parse encoder data from Pico.
    Expected format: ENC:left:right
    """
    try:
        parts = line.split(':')
        left = int(parts[1])
        right = int(parts[2])
        return left, right
    except Exception:
        return 0, 0
