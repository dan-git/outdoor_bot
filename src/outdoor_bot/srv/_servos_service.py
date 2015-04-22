"""autogenerated by genpy from outdoor_bot/servos_serviceRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class servos_serviceRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "outdoor_bot/servos_serviceRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(servos_serviceRequest, self).__init__(*args, **kwds)

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
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from outdoor_bot/servos_serviceResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class servos_serviceResponse(genpy.Message):
  _md5sum = "f6ec05adc7477127aea3baa38fdf4033"
  _type = "outdoor_bot/servos_serviceResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 FrontDigcamPan
int32 FrontDigcamTilt
int32 RearDigcamPan
int32 RearDigcamTilt
int32 FrontWebcamPan
int32 FrontWebcamTilt
int32 RearWebcamPan
int32 RearWebcamTilt


"""
  __slots__ = ['FrontDigcamPan','FrontDigcamTilt','RearDigcamPan','RearDigcamTilt','FrontWebcamPan','FrontWebcamTilt','RearWebcamPan','RearWebcamTilt']
  _slot_types = ['int32','int32','int32','int32','int32','int32','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       FrontDigcamPan,FrontDigcamTilt,RearDigcamPan,RearDigcamTilt,FrontWebcamPan,FrontWebcamTilt,RearWebcamPan,RearWebcamTilt

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(servos_serviceResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.FrontDigcamPan is None:
        self.FrontDigcamPan = 0
      if self.FrontDigcamTilt is None:
        self.FrontDigcamTilt = 0
      if self.RearDigcamPan is None:
        self.RearDigcamPan = 0
      if self.RearDigcamTilt is None:
        self.RearDigcamTilt = 0
      if self.FrontWebcamPan is None:
        self.FrontWebcamPan = 0
      if self.FrontWebcamTilt is None:
        self.FrontWebcamTilt = 0
      if self.RearWebcamPan is None:
        self.RearWebcamPan = 0
      if self.RearWebcamTilt is None:
        self.RearWebcamTilt = 0
    else:
      self.FrontDigcamPan = 0
      self.FrontDigcamTilt = 0
      self.RearDigcamPan = 0
      self.RearDigcamTilt = 0
      self.FrontWebcamPan = 0
      self.FrontWebcamTilt = 0
      self.RearWebcamPan = 0
      self.RearWebcamTilt = 0

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
      _x = self
      buff.write(_struct_8i.pack(_x.FrontDigcamPan, _x.FrontDigcamTilt, _x.RearDigcamPan, _x.RearDigcamTilt, _x.FrontWebcamPan, _x.FrontWebcamTilt, _x.RearWebcamPan, _x.RearWebcamTilt))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.FrontDigcamPan, _x.FrontDigcamTilt, _x.RearDigcamPan, _x.RearDigcamTilt, _x.FrontWebcamPan, _x.FrontWebcamTilt, _x.RearWebcamPan, _x.RearWebcamTilt,) = _struct_8i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_8i.pack(_x.FrontDigcamPan, _x.FrontDigcamTilt, _x.RearDigcamPan, _x.RearDigcamTilt, _x.FrontWebcamPan, _x.FrontWebcamTilt, _x.RearWebcamPan, _x.RearWebcamTilt))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.FrontDigcamPan, _x.FrontDigcamTilt, _x.RearDigcamPan, _x.RearDigcamTilt, _x.FrontWebcamPan, _x.FrontWebcamTilt, _x.RearWebcamPan, _x.RearWebcamTilt,) = _struct_8i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_8i = struct.Struct("<8i")
class servos_service(object):
  _type          = 'outdoor_bot/servos_service'
  _md5sum = 'f6ec05adc7477127aea3baa38fdf4033'
  _request_class  = servos_serviceRequest
  _response_class = servos_serviceResponse
