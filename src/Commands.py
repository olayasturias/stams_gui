import rospy
import bitstring


class Command(object):
    """
    Sonar commands
    """
    def __init__(self, id, payload = None, command = None):
        """
        Construct command object
        :param id:
        :param payload:
        """

        self.id = id

        self.payload = payload if payload else bitstring.BitStream()

        self.command = command if command else bitstring.BitStream()

    def serialize(self):
        """
        Construct string of bytes to send to sonar
        :return:
        """

        # The len must be multiple of 4 bits to convert unambiguously

        id_len = self.id.bit_length()
        while (id_len % 4)!= 0:
            id_len += 1
        if self.payload:
            pay_len = self.payload.bit_length()
            while (pay_len % 4)!= 0:
                pay_len += 1
        else: pay_len = 0
        if self.command:
            com_len = self.command.bit_length()
            while (com_len % 4)!= 0:
                com_len += 1
        else: com_len = 0

        values = {
            "id": self.id,
            "id_len": id_len,
            "payload": self.payload,
            "payload_len": pay_len,
            "command": self.command,
            "command_len": com_len
        }


        if self.id == Message.MEASURE or self.id == Message.SINGLE_MEASURE:
            serial_format = (
                "uint:id_len=id, bits:payload_len=payload, bits:command_len = command, 0x0D0A"
            )
        else:
            serial_format = (
                "0x23, uint:id_len=id, bits:payload_len=payload, bits:command_len = command, 0x0D0A"
            )

        message = bitstring.pack(serial_format, **values)

        rospy.logdebug("Sent command '0x%s'", message.hex)

        return message.tobytes()
