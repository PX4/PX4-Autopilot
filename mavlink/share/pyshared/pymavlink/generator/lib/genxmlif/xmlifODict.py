from types    import DictType
from UserDict import UserDict

class odict(UserDict):
    def __init__(self, dictOrTuple = None):
        self._keys = []
        UserDict.__init__(self, dictOrTuple)

    def __delitem__(self, key):
        UserDict.__delitem__(self, key)
        self._keys.remove(key)

    def __setitem__(self, key, item):
        UserDict.__setitem__(self, key, item)
        if key not in self._keys: self._keys.append(key)

    def clear(self):
        UserDict.clear(self)
        self._keys = []

    def copy(self):
        newInstance = odict()
        newInstance.update(self)
        return newInstance

    def items(self):
        return zip(self._keys, self.values())

    def keys(self):
        return self._keys[:]

    def popitem(self):
        try:
            key = self._keys[-1]
        except IndexError:
            raise KeyError('dictionary is empty')

        val = self[key]
        del self[key]

        return (key, val)

    def setdefault(self, key, failobj = None):
        if key not in self._keys: 
            self._keys.append(key)
        return UserDict.setdefault(self, key, failobj)

    def update(self, dictOrTuple):
        if isinstance(dictOrTuple, DictType):
            itemList = dictOrTuple.items()
        else:
            itemList = dictOrTuple
        for key, val in itemList:
            self.__setitem__(key,val)

    def values(self):
        return map(self.get, self._keys)