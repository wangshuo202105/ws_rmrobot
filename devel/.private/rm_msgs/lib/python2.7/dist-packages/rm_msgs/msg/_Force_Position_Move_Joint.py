# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rm_msgs/Force_Position_Move_Joint.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Force_Position_Move_Joint(genpy.Message):
  _md5sum = "289142d2c6dc6949e95e7d291c27112f"
  _type = "rm_msgs/Force_Position_Move_Joint"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32[6] joint
uint8 sensor
uint8 mode
uint8 dir
int16 force
"""
  __slots__ = ['joint','sensor','mode','dir','force']
  _slot_types = ['float32[6]','uint8','uint8','uint8','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint,sensor,mode,dir,force

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Force_Position_Move_Joint, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.joint is None:
        self.joint = [0.] * 6
      if self.sensor is None:
        self.sensor = 0
      if self.mode is None:
        self.mode = 0
      if self.dir is None:
        self.dir = 0
      if self.force is None:
        self.force = 0
    else:
      self.joint = [0.] * 6
      self.sensor = 0
      self.mode = 0
      self.dir = 0
      self.force = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_get_struct_6f().pack(*self.joint))
      _x = self
      buff.write(_get_struct_3Bh().pack(_x.sensor, _x.mode, _x.dir, _x.force))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 24
      self.joint = _get_struct_6f().unpack(str[start:end])
      _x = self
      start = end
      end += 5
      (_x.sensor, _x.mode, _x.dir, _x.force,) = _get_struct_3Bh().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.joint.tostring())
      _x = self
      buff.write(_get_struct_3Bh().pack(_x.sensor, _x.mode, _x.dir, _x.force))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 24
      self.joint = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=6)
      _x = self
      start = end
      end += 5
      (_x.sensor, _x.mode, _x.dir, _x.force,) = _get_struct_3Bh().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3Bh = None
def _get_struct_3Bh():
    global _struct_3Bh
    if _struct_3Bh is None:
        _struct_3Bh = struct.Struct("<3Bh")
    return _struct_3Bh
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
