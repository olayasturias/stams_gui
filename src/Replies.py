class Reply(object):
    """
    Parses and verifies reply packages
    """
    def __init__(self, bitstream, id = 0):
        """

        :param bitstream:
        """
        self.bitstream = bitstream
        self.id = id
        self.name = Message.to_string(self.id)
        self.payload = None
        self.dataformat = 'NMEA'
        self.dataunits = None

        self.parse()

    def parse(self):
        """
        Parses packet into header, message ID and payload
        :return:
        """
        try:
            if self.bitstream:
                # Parse message header
                self.bitstream.bytepos = 0

                if self.bitstream.endswith("\n"):
                    pass

                else:
                    raise PacketIncomplete("Packet does not end with carriage return")

                if self.bitstream.find('0x 50 52 56 41 54',bytealigned=True): # If 'PRVAT' text in bitstream
                    self.dataformat = 'NMEA'
                else:
                    self.dataformat = 'TRITECH'

                if self.dataformat=='NMEA' and self.id != Message.CONFIGURATION_PARAM:
                    # go to first comma
                    self.bitstream.bytepos = self.bitstream.find('0x2C', bytealigned = True)[0]/8 + 1
                    self.payload = self.bitstream.read('bytes:6')
                    #skip comma
                    self.bitstream.read('bytes:1')
                    self.dataunits = self.bitstream.read('bytes:1')


                elif self.dataformat=='TRITECH' and self.id != Message.CONFIGURATION_PARAM:
                    self.bitstream.bytepos = 0
                    self.payload = self.bitstream.read('bytes:6')
                    self.dataunits = self.bitstream.read('bytes:1')
                else:
                    self.bitstream.bytepos = 0
                    length_string = 'bytes:'+ str(len(self.bitstream)/8)
                    self.payload = self.bitstream.read(length_string)

            else:
                pass

        except ValueError as e:
            raise PacketCorrupted("Unexpected error", e)


