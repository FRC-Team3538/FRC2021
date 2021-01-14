# automatically generated by the FlatBuffers compiler, do not modify

# namespace: rj

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class PCMStatusFrame(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsPCMStatusFrame(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = PCMStatusFrame()
        x.Init(buf, n + offset)
        return x

    # PCMStatusFrame
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # PCMStatusFrame
    def Module(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Int32Flags, o + self._tab.Pos)
        return 0

    # PCMStatusFrame
    def Enabled(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

    # PCMStatusFrame
    def PressureSwitchValve(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

    # PCMStatusFrame
    def CompressorCurrent(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # PCMStatusFrame
    def ClosedLoopControl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

    # PCMStatusFrame
    def CompressorCurrentTooHighFault(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

    # PCMStatusFrame
    def CompressorShortedFault(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(16))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

    # PCMStatusFrame
    def CompressorNotConnectedFault(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(18))
        if o != 0:
            return bool(self._tab.Get(flatbuffers.number_types.BoolFlags, o + self._tab.Pos))
        return False

def PCMStatusFrameStart(builder): builder.StartObject(8)
def PCMStatusFrameAddModule(builder, module): builder.PrependInt32Slot(0, module, 0)
def PCMStatusFrameAddEnabled(builder, enabled): builder.PrependBoolSlot(1, enabled, 0)
def PCMStatusFrameAddPressureSwitchValve(builder, pressureSwitchValve): builder.PrependBoolSlot(2, pressureSwitchValve, 0)
def PCMStatusFrameAddCompressorCurrent(builder, compressorCurrent): builder.PrependFloat64Slot(3, compressorCurrent, 0.0)
def PCMStatusFrameAddClosedLoopControl(builder, closedLoopControl): builder.PrependBoolSlot(4, closedLoopControl, 0)
def PCMStatusFrameAddCompressorCurrentTooHighFault(builder, compressorCurrentTooHighFault): builder.PrependBoolSlot(5, compressorCurrentTooHighFault, 0)
def PCMStatusFrameAddCompressorShortedFault(builder, compressorShortedFault): builder.PrependBoolSlot(6, compressorShortedFault, 0)
def PCMStatusFrameAddCompressorNotConnectedFault(builder, compressorNotConnectedFault): builder.PrependBoolSlot(7, compressorNotConnectedFault, 0)
def PCMStatusFrameEnd(builder): return builder.EndObject()
