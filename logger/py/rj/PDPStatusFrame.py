# automatically generated by the FlatBuffers compiler, do not modify

# namespace: rj

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class PDPStatusFrame(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsPDPStatusFrame(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = PDPStatusFrame()
        x.Init(buf, n + offset)
        return x

    # PDPStatusFrame
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # PDPStatusFrame
    def Module(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Int32Flags, o + self._tab.Pos)
        return 0

    # PDPStatusFrame
    def Voltage(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # PDPStatusFrame
    def Temperature(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # PDPStatusFrame
    def ChannelCurrent(self, j):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            a = self._tab.Vector(o)
            return self._tab.Get(flatbuffers.number_types.Float64Flags, a + flatbuffers.number_types.UOffsetTFlags.py_type(j * 8))
        return 0

    # PDPStatusFrame
    def ChannelCurrentAsNumpy(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.GetVectorAsNumpy(flatbuffers.number_types.Float64Flags, o)
        return 0

    # PDPStatusFrame
    def ChannelCurrentLength(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.VectorLen(o)
        return 0

    # PDPStatusFrame
    def ChannelCurrentIsNone(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        return o == 0

    # PDPStatusFrame
    def TotalCurrent(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # PDPStatusFrame
    def TotalPower(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # PDPStatusFrame
    def TotalEnergy(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(16))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

def PDPStatusFrameStart(builder): builder.StartObject(7)
def PDPStatusFrameAddModule(builder, module): builder.PrependInt32Slot(0, module, 0)
def PDPStatusFrameAddVoltage(builder, voltage): builder.PrependFloat64Slot(1, voltage, 0.0)
def PDPStatusFrameAddTemperature(builder, temperature): builder.PrependFloat64Slot(2, temperature, 0.0)
def PDPStatusFrameAddChannelCurrent(builder, channelCurrent): builder.PrependUOffsetTRelativeSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(channelCurrent), 0)
def PDPStatusFrameStartChannelCurrentVector(builder, numElems): return builder.StartVector(8, numElems, 8)
def PDPStatusFrameAddTotalCurrent(builder, totalCurrent): builder.PrependFloat64Slot(4, totalCurrent, 0.0)
def PDPStatusFrameAddTotalPower(builder, totalPower): builder.PrependFloat64Slot(5, totalPower, 0.0)
def PDPStatusFrameAddTotalEnergy(builder, totalEnergy): builder.PrependFloat64Slot(6, totalEnergy, 0.0)
def PDPStatusFrameEnd(builder): return builder.EndObject()
