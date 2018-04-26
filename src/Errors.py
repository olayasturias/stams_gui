class SonarNotFound(Exception):
    """Sonar port could not be found."""
    pass

class DataNotSent(Exception):
    """Sonar is not sending information."""
    pass

class PacketIncomplete(Exception):
    """Packet is incomplete."""
    pass

class PacketCorrupted(Exception):
    """Packet is corrupt."""
    pass

class SonarNotConfigured(Exception):
    """Sonar is not configured for scanning."""
    pass

class TimeoutError(Exception):
    """Communication timed out."""
    pass

