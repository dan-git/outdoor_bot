"""autogenerated by genpy from outdoor_bot/radar_serviceRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class radar_serviceRequest(genpy.Message):
  _md5sum = "67e45d87e676eebe909bb231289531ad"
  _type = "outdoor_bot/radar_serviceRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool enableRadarData

"""
  __slots__ = ['enableRadarData']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       enableRadarData

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(radar_serviceRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.enableRadarData is None:
        self.enableRadarData = False
    else:
      self.enableRadarData = False

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
      buff.write(_struct_B.pack(self.enableRadarData))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.enableRadarData,) = _struct_B.unpack(str[start:end])
      self.enableRadarData = bool(self.enableRadarData)
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
      buff.write(_struct_B.pack(self.enableRadarData))
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
      start = end
      end += 1
      (self.enableRadarData,) = _struct_B.unpack(str[start:end])
      self.enableRadarData = bool(self.enableRadarData)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
"""autogenerated by genpy from outdoor_bot/radar_serviceResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class radar_serviceResponse(genpy.Message):
  _md5sum = "8d82d05f0eb51c0f2c54b2f67664b553"
  _type = "outdoor_bot/radar_serviceResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 distanceToHomeLeft
float32 distanceToHomeCenter
float32 distanceToHomeRight
float32 deltaDistanceToHome
float32 velocityFromHome


"""
  __slots__ = ['distanceToHomeLeft','distanceToHomeCenter','distanceToHomeRight','deltaDistanceToHome','velocityFromHome']
  _slot_types = ['float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       distanceToHomeLeft,distanceToHomeCenter,distanceToHomeRight,deltaDistanceToHome,velocityFromHome

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(radar_serviceResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.distanceToHomeLeft is None:
        self.distanceToHomeLeft = 0.
      if self.distanceToHomeCenter is None:
        self.distanceToHomeCenter = 0.
      if self.distanceToHomeRight is None:
        self.distanceToHomeRight = 0.
      if self.deltaDistanceToHome is None:
        self.deltaDistanceToHome = 0.
      if self.velocityFromHome is None:
        self.velocityFromHome = 0.
    else:
      self.distanceToHomeLeft = 0.
      self.distanceToHomeCenter = 0.
      self.distanceToHomeRight = 0.
      self.deltaDistanceToHome = 0.
      self.velocityFromHome = 0.

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
      buff.write(_struct_5f.pack(_x.distanceToHomeLeft, _x.distanceToHomeCenter, _x.distanceToHomeRight, _x.deltaDistanceToHome, _x.velocityFromHome))
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
      end += 20
      (_x.distanceToHomeLeft, _x.distanceToHomeCenter, _x.distanceToHomeRight, _x.deltaDistanceToHome, _x.velocityFromHome,) = _struct_5f.unpack(str[start:end])
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
      buff.write(_struct_5f.pack(_x.distanceToHomeLeft, _x.distanceToHomeCenter, _x.distanceToHomeRight, _x.deltaDistanceToHome, _x.velocityFromHome))
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
      end += 20
      (_x.distanceToHomeLeft, _x.distanceToHomeCenter, _x.distanceToHomeRight, _x.deltaDistanceToHome, _x.velocityFromHome,) = _struct_5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5f = struct.Struct("<5f")
class radar_service(object):
  _type          = 'outdoor_bot/radar_service'
  _md5sum = 'ac154965ece8b4305addd6db6f9bfb4d'
  _request_class  = radar_serviceRequest
  _response_class = radar_serviceResponse
