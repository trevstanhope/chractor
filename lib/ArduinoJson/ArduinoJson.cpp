// WARNING:
// --------
// This file is a workaround for old version of the Arduino IDE.
// If you are using Arduino IDE 1.0.6 or above, then you installed the wrong 
// package of ArduinoJson.
// In that case, just delete the current installation and install the package.


// src/Internals/Comments.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/Comments.hpp"

inline static const char *skipCStyleComment(const char *ptr) {
  ptr += 2;
  for (;;) {
    if (ptr[0] == '\0') return ptr;
    if (ptr[0] == '*' && ptr[1] == '/') return ptr + 2;
    ptr++;
  }
}

inline static const char *skipCppStyleComment(const char *ptr) {
  ptr += 2;
  for (;;) {
    if (ptr[0] == '\0' || ptr[0] == '\n') return ptr;
    ptr++;
  }
}

const char *ArduinoJson::Internals::skipSpacesAndComments(const char *ptr) {
  for (;;) {
    switch (ptr[0]) {
      case ' ':
      case '\t':
      case '\r':
      case '\n':
        ptr++;
        continue;
      case '/':
        switch (ptr[1]) {
          case '*':
            ptr = skipCStyleComment(ptr);
            break;
          case '/':
            ptr = skipCppStyleComment(ptr);
            break;
          default:
            return ptr;
        }
        break;
      default:
        return ptr;
    }
  }
}

// src/Internals/Encoding.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/Encoding.hpp"

// How to escape special chars:
// _escapeTable[2*i+1] => the special char
// _escapeTable[2*i] => the char to use instead
const char ArduinoJson::Internals::Encoding::_escapeTable[] =
    "\"\"\\\\b\bf\fn\nr\rt\t";

// src/Internals/IndentedPrint.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/IndentedPrint.hpp"

using namespace ArduinoJson::Internals;

size_t IndentedPrint::write(uint8_t c) {
  size_t n = 0;

  if (isNewLine) n += writeTabs();

  n += sink->write(c);

  isNewLine = c == '\n';

  return n;
}

inline size_t IndentedPrint::writeTabs() {
  size_t n = 0;

  for (int i = 0; i < level * tabSize; i++) n += sink->write(' ');

  return n;
}

// src/Internals/JsonParser.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/JsonParser.hpp"

#include "include/ArduinoJson/Internals/Comments.hpp"
#include "include/ArduinoJson/Internals/Encoding.hpp"
#include "include/ArduinoJson/JsonArray.hpp"
#include "include/ArduinoJson/JsonBuffer.hpp"
#include "include/ArduinoJson/JsonObject.hpp"

using namespace ArduinoJson;
using namespace ArduinoJson::Internals;

bool JsonParser::skip(char charToSkip) {
  const char *ptr = skipSpacesAndComments(_readPtr);
  if (*ptr != charToSkip) return false;
  ptr++;
  _readPtr = skipSpacesAndComments(ptr);
  return true;
}

bool JsonParser::parseAnythingTo(JsonVariant *destination) {
  if (_nestingLimit == 0) return false;
  _nestingLimit--;
  bool success = parseAnythingToUnsafe(destination);
  _nestingLimit++;
  return success;
}

inline bool JsonParser::parseAnythingToUnsafe(JsonVariant *destination) {
  _readPtr = skipSpacesAndComments(_readPtr);

  switch (*_readPtr) {
    case '[':
      return parseArrayTo(destination);

    case '{':
      return parseObjectTo(destination);

    default:
      return parseStringTo(destination);
  }
}

JsonArray &JsonParser::parseArray() {
  // Create an empty array
  JsonArray &array = _buffer->createArray();

  // Check opening braket
  if (!skip('[')) goto ERROR_MISSING_BRACKET;
  if (skip(']')) goto SUCCESS_EMPTY_ARRAY;

  // Read each value
  for (;;) {
    // 1 - Parse value
    JsonVariant value;
    if (!parseAnythingTo(&value)) goto ERROR_INVALID_VALUE;
    if (!array.add(value)) goto ERROR_NO_MEMORY;

    // 2 - More values?
    if (skip(']')) goto SUCCES_NON_EMPTY_ARRAY;
    if (!skip(',')) goto ERROR_MISSING_COMMA;
  }

SUCCESS_EMPTY_ARRAY:
SUCCES_NON_EMPTY_ARRAY:
  return array;

ERROR_INVALID_VALUE:
ERROR_MISSING_BRACKET:
ERROR_MISSING_COMMA:
ERROR_NO_MEMORY:
  return JsonArray::invalid();
}

bool JsonParser::parseArrayTo(JsonVariant *destination) {
  JsonArray &array = parseArray();
  if (!array.success()) return false;

  *destination = array;
  return true;
}

JsonObject &JsonParser::parseObject() {
  // Create an empty object
  JsonObject &object = _buffer->createObject();

  // Check opening brace
  if (!skip('{')) goto ERROR_MISSING_BRACE;
  if (skip('}')) goto SUCCESS_EMPTY_OBJECT;

  // Read each key value pair
  for (;;) {
    // 1 - Parse key
    const char *key = parseString();
    if (!key) goto ERROR_INVALID_KEY;
    if (!skip(':')) goto ERROR_MISSING_COLON;

    // 2 - Parse value
    JsonVariant value;
    if (!parseAnythingTo(&value)) goto ERROR_INVALID_VALUE;
    if (!object.set(key, value)) goto ERROR_NO_MEMORY;

    // 3 - More keys/values?
    if (skip('}')) goto SUCCESS_NON_EMPTY_OBJECT;
    if (!skip(',')) goto ERROR_MISSING_COMMA;
  }

SUCCESS_EMPTY_OBJECT:
SUCCESS_NON_EMPTY_OBJECT:
  return object;

ERROR_INVALID_KEY:
ERROR_INVALID_VALUE:
ERROR_MISSING_BRACE:
ERROR_MISSING_COLON:
ERROR_MISSING_COMMA:
ERROR_NO_MEMORY:
  return JsonObject::invalid();
}

bool JsonParser::parseObjectTo(JsonVariant *destination) {
  JsonObject &object = parseObject();
  if (!object.success()) return false;

  *destination = object;
  return true;
}

static inline bool isInRange(char c, char min, char max) {
  return min <= c && c <= max;
}

static inline bool isLetterOrNumber(char c) {
  return isInRange(c, '0', '9') || isInRange(c, 'a', 'z') ||
         isInRange(c, 'A', 'Z') || c == '-' || c == '.';
}

static inline bool isQuote(char c) { return c == '\'' || c == '\"'; }

const char *JsonParser::parseString() {
  const char *readPtr = _readPtr;
  char *writePtr = _writePtr;

  char c = *readPtr;

  if (isQuote(c)) {  // quotes
    char stopChar = c;
    for (;;) {
      c = *++readPtr;
      if (c == '\0') break;

      if (c == stopChar) {
        readPtr++;
        break;
      }

      if (c == '\\') {
        // replace char
        c = Encoding::unescapeChar(*++readPtr);
        if (c == '\0') break;
      }

      *writePtr++ = c;
    }
  } else {  // no quotes
    for (;;) {
      if (!isLetterOrNumber(c)) break;
      *writePtr++ = c;
      c = *++readPtr;
    }
  }
  // end the string here
  *writePtr++ = '\0';

  const char *startPtr = _writePtr;

  // update end ptr
  _readPtr = readPtr;
  _writePtr = writePtr;

  // return pointer to unquoted string
  return startPtr;
}

bool JsonParser::parseStringTo(JsonVariant *destination) {
  bool hasQuotes = isQuote(_readPtr[0]);
  const char *value = parseString();
  if (value == NULL) return false;
  if (hasQuotes) {
    *destination = value;
  } else {
    *destination = RawJson(value);
  }
  return true;
}

// src/Internals/List.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/List.hpp"

#include "include/ArduinoJson/JsonPair.hpp"
#include "include/ArduinoJson/JsonVariant.hpp"

using namespace ArduinoJson;
using namespace ArduinoJson::Internals;

template <typename T>
size_t List<T>::size() const {
  size_t nodeCount = 0;
  for (node_type *node = _firstNode; node; node = node->next) nodeCount++;
  return nodeCount;
}

template <typename T>
typename List<T>::node_type *List<T>::addNewNode() {
  node_type *newNode = new (_buffer) node_type();

  if (_firstNode) {
    node_type *lastNode = _firstNode;
    while (lastNode->next) lastNode = lastNode->next;
    lastNode->next = newNode;
  } else {
    _firstNode = newNode;
  }

  return newNode;
}

template <typename T>
void List<T>::removeNode(node_type *nodeToRemove) {
  if (!nodeToRemove) return;
  if (nodeToRemove == _firstNode) {
    _firstNode = nodeToRemove->next;
  } else {
    for (node_type *node = _firstNode; node; node = node->next)
      if (node->next == nodeToRemove) node->next = nodeToRemove->next;
  }
}

template class ArduinoJson::Internals::List<JsonPair>;
template class ArduinoJson::Internals::List<JsonVariant>;

// src/Internals/Prettyfier.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/Prettyfier.hpp"

using namespace ArduinoJson::Internals;

size_t Prettyfier::write(uint8_t c) {
  size_t n = _inString ? handleStringChar(c) : handleMarkupChar(c);
  _previousChar = c;
  return n;
}

inline size_t Prettyfier::handleStringChar(uint8_t c) {
  bool isQuote = c == '"' && _previousChar != '\\';

  if (isQuote) _inString = false;

  return _sink.write(c);
}

inline size_t Prettyfier::handleMarkupChar(uint8_t c) {
  switch (c) {
    case '{':
    case '[':
      return handleBlockOpen(c);

    case '}':
    case ']':
      return handleBlockClose(c);

    case ':':
      return handleColon();

    case ',':
      return handleComma();

    case '"':
      return handleQuoteOpen();

    default:
      return handleNormalChar(c);
  }
}

inline size_t Prettyfier::handleBlockOpen(uint8_t c) {
  return indentIfNeeded() + _sink.write(c);
}

inline size_t Prettyfier::handleBlockClose(uint8_t c) {
  return unindentIfNeeded() + _sink.write(c);
}

inline size_t Prettyfier::handleColon() {
  return _sink.write(':') + _sink.write(' ');
}

inline size_t Prettyfier::handleComma() {
  return _sink.write(',') + _sink.println();
}

inline size_t Prettyfier::handleQuoteOpen() {
  _inString = true;
  return indentIfNeeded() + _sink.write('"');
}

inline size_t Prettyfier::handleNormalChar(uint8_t c) {
  return indentIfNeeded() + _sink.write(c);
}

size_t Prettyfier::indentIfNeeded() {
  if (!inEmptyBlock()) return 0;

  _sink.indent();
  return _sink.println();
}

size_t Prettyfier::unindentIfNeeded() {
  if (inEmptyBlock()) return 0;

  _sink.unindent();
  return _sink.println();
}

// src/Internals/StaticStringBuilder.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/Internals/StaticStringBuilder.hpp"

using namespace ArduinoJson::Internals;

size_t StaticStringBuilder::write(uint8_t c) {
  if (length >= capacity) return 0;

  buffer[length++] = c;
  buffer[length] = '\0';
  return 1;
}

// src/JsonArray.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/JsonArray.hpp"

#include "include/ArduinoJson/JsonBuffer.hpp"
#include "include/ArduinoJson/JsonObject.hpp"

using namespace ArduinoJson;
using namespace ArduinoJson::Internals;

JsonArray JsonArray::_invalid(NULL);

JsonArray::node_type *JsonArray::getNodeAt(size_t index) const {
  node_type *node = _firstNode;
  while (node && index--) node = node->next;
  return node;
}

void JsonArray::removeAt(size_t index) { removeNode(getNodeAt(index)); }

void JsonArray::writeTo(JsonWriter &writer) const {
  writer.beginArray();

  const node_type *child = _firstNode;
  while (child) {
    child->content.writeTo(writer);

    child = child->next;
    if (!child) break;

    writer.writeComma();
  }

  writer.endArray();
}

// src/JsonBuffer.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/JsonBuffer.hpp"

#include "include/ArduinoJson/Internals/JsonParser.hpp"
#include "include/ArduinoJson/JsonArray.hpp"
#include "include/ArduinoJson/JsonObject.hpp"

using namespace ArduinoJson;
using namespace ArduinoJson::Internals;

JsonArray &JsonBuffer::createArray() {
  JsonArray *ptr = new (this) JsonArray(this);
  return ptr ? *ptr : JsonArray::invalid();
}

JsonObject &JsonBuffer::createObject() {
  JsonObject *ptr = new (this) JsonObject(this);
  return ptr ? *ptr : JsonObject::invalid();
}

JsonArray &JsonBuffer::parseArray(char *json, uint8_t nestingLimit) {
  JsonParser parser(this, json, nestingLimit);
  return parser.parseArray();
}

JsonObject &JsonBuffer::parseObject(char *json, uint8_t nestingLimit) {
  JsonParser parser(this, json, nestingLimit);
  return parser.parseObject();
}

char *JsonBuffer::strdup(const char *source, size_t length) {
  size_t size = length + 1;
  char *dest = static_cast<char *>(alloc(size));
  if (dest != NULL) memcpy(dest, source, size);
  return dest;
}

// src/JsonObject.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/JsonObject.hpp"

#include <string.h>  // for strcmp

#include "include/ArduinoJson/Internals/StaticStringBuilder.hpp"
#include "include/ArduinoJson/JsonArray.hpp"
#include "include/ArduinoJson/JsonBuffer.hpp"

using namespace ArduinoJson;
using namespace ArduinoJson::Internals;

JsonObject JsonObject::_invalid(NULL);

JsonObject::node_type *JsonObject::getNodeAt(const char *key) const {
  for (node_type *node = _firstNode; node; node = node->next) {
    if (!strcmp(node->content.key, key)) return node;
  }
  return NULL;
}

void JsonObject::writeTo(JsonWriter &writer) const {
  writer.beginObject();

  const node_type *node = _firstNode;
  while (node) {
    writer.writeString(node->content.key);
    writer.writeColon();
    node->content.value.writeTo(writer);

    node = node->next;
    if (!node) break;

    writer.writeComma();
  }

  writer.endObject();
}

// src/JsonVariant.cpp
//
// Copyright Benoit Blanchon 2014-2016
// MIT License
//
// Arduino JSON library
// https://github.com/bblanchon/ArduinoJson
// If you like this project, please add a star!

#include "include/ArduinoJson/JsonVariant.hpp"

#include "include/ArduinoJson/JsonArray.hpp"
#include "include/ArduinoJson/JsonObject.hpp"

#include <errno.h>   // for errno
#include <stdlib.h>  // for strtol, strtod

using namespace ArduinoJson::Internals;

namespace ArduinoJson {

const char *JsonVariant::asString() const {
  if (_type == JSON_UNPARSED && _content.asString &&
      !strcmp("null", _content.asString))
    return NULL;
  if (_type == JSON_STRING || _type == JSON_UNPARSED) return _content.asString;
  return NULL;
}

JsonFloat JsonVariant::asFloat() const {
  if (_type >= JSON_FLOAT_0_DECIMALS) return _content.asFloat;

  if (_type == JSON_INTEGER || _type == JSON_BOOLEAN)
    return static_cast<JsonFloat>(_content.asInteger);

  if ((_type == JSON_STRING || _type == JSON_UNPARSED) && _content.asString)
    return parse<JsonFloat>(_content.asString);

  return 0.0;
}

String JsonVariant::toString() const {
  String s;
  if ((_type == JSON_STRING || _type == JSON_UNPARSED) &&
      _content.asString != NULL)
    s = _content.asString;
  else
    printTo(s);
  return s;
}

bool JsonVariant::isBoolean() const {
  if (_type == JSON_BOOLEAN) return true;

  if (_type != JSON_UNPARSED || _content.asString == NULL) return false;

  return !strcmp(_content.asString, "true") ||
         !strcmp(_content.asString, "false");
}

bool JsonVariant::isInteger() const {
  if (_type == JSON_INTEGER) return true;

  if (_type != JSON_UNPARSED || _content.asString == NULL) return false;

  char *end;
  errno = 0;
  strtol(_content.asString, &end, 10);

  return *end == '\0' && errno == 0;
}

bool JsonVariant::isFloat() const {
  if (_type >= JSON_FLOAT_0_DECIMALS) return true;

  if (_type != JSON_UNPARSED || _content.asString == NULL) return false;

  char *end;
  errno = 0;
  strtod(_content.asString, &end);

  return *end == '\0' && errno == 0 && !is<long>();
}

void JsonVariant::writeTo(JsonWriter &writer) const {
  if (_type == JSON_ARRAY)
    _content.asArray->writeTo(writer);

  else if (_type == JSON_OBJECT)
    _content.asObject->writeTo(writer);

  else if (_type == JSON_STRING)
    writer.writeString(_content.asString);

  else if (_type == JSON_UNPARSED)
    writer.writeRaw(_content.asString);

  else if (_type == JSON_INTEGER)
    writer.writeInteger(_content.asInteger);

  else if (_type == JSON_BOOLEAN)
    writer.writeBoolean(_content.asInteger != 0);

  else if (_type >= JSON_FLOAT_0_DECIMALS) {
    uint8_t decimals = static_cast<uint8_t>(_type - JSON_FLOAT_0_DECIMALS);
    writer.writeFloat(_content.asFloat, decimals);
  }
}
}
