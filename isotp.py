"""ISO-TP single-frame and flow-controlled multi-frame transmission."""

from typing import Optional

from slcan_protocol import CAN_PADDING_BYTE, CanFrame


class IsoTpSender:
    def __init__(
        self, arbitration_id: int, flow_control_id: int, payload: bytes
    ) -> None:
        if not 1 <= len(payload) <= 0xFFF:
            raise ValueError("ISO-TP payload length must be between 1 and 4095 bytes.")
        if not 0 <= arbitration_id <= 0x7FF or not 0 <= flow_control_id <= 0x7FF:
            raise ValueError("ISO-TP CAN IDs must be 11-bit identifiers.")
        self.arbitration_id = arbitration_id
        self.flow_control_id = flow_control_id
        self.payload = payload
        self._offset = min(len(payload), 6)
        self._sequence_number = 1
        self._block_size = 0
        self._frames_in_block = 0
        self._wait_frames = 0
        self._waiting_for_flow_control = len(payload) > 7
        self._complete = False
        self.error: Optional[str] = None
        self.st_min_ms = 0

    @property
    def waiting_for_flow_control(self) -> bool:
        return self._waiting_for_flow_control

    @property
    def complete(self) -> bool:
        return self._complete

    def initial_frame(self) -> CanFrame:
        if len(self.payload) <= 7:
            data = bytes((len(self.payload),)) + self.payload
            self._complete = True
        else:
            length = len(self.payload)
            data = bytes((0x10 | (length >> 8), length & 0xFF)) + self.payload[:6]
        data += bytes((CAN_PADDING_BYTE,)) * (8 - len(data))
        return self._frame(data)

    def accept_flow_control(self, frame: CanFrame) -> bool:
        if (
            self._complete
            or not self._waiting_for_flow_control
            or frame.extended
            or frame.remote
            or frame.arbitration_id != self.flow_control_id
            or frame.dlc < 3
            or len(frame.data) < 3
            or frame.data[0] >> 4 != 0x3
        ):
            return False

        flow_status = frame.data[0] & 0x0F
        if flow_status == 0x01:
            self._wait_frames += 1
            if self._wait_frames > 3:
                self._fail("Tester exceeded the ISO-TP wait-frame limit.")
            return True
        if flow_status == 0x02:
            self._fail("Tester reported ISO-TP buffer overflow.")
            return True
        if flow_status != 0:
            return False

        separation = frame.data[2]
        if separation <= 0x7F:
            self.st_min_ms = separation
        elif 0xF1 <= separation <= 0xF9:
            self.st_min_ms = 1
        else:
            self._fail("Tester supplied a reserved ISO-TP separation time.")
            return True

        self._block_size = frame.data[1]
        self._frames_in_block = 0
        self._wait_frames = 0
        self._waiting_for_flow_control = False
        return True

    def next_frame(self) -> Optional[CanFrame]:
        if self._complete or self._waiting_for_flow_control:
            return None

        chunk = self.payload[self._offset : self._offset + 7]
        if not chunk:
            self._complete = True
            return None
        data = bytes((0x20 | self._sequence_number,)) + chunk
        data += bytes((CAN_PADDING_BYTE,)) * (8 - len(data))
        self._offset += len(chunk)
        self._sequence_number = (self._sequence_number + 1) & 0x0F
        self._frames_in_block += 1

        if self._offset >= len(self.payload):
            self._complete = True
        elif self._block_size and self._frames_in_block >= self._block_size:
            self._waiting_for_flow_control = True
        return self._frame(data)

    def _frame(self, data: bytes) -> CanFrame:
        return CanFrame(
            arbitration_id=self.arbitration_id,
            extended=False,
            remote=False,
            dlc=8,
            data=data,
        )

    def _fail(self, message: str) -> None:
        self.error = message
        self._complete = True
        self._waiting_for_flow_control = False
