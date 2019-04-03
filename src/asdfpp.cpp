/*******************************************************************************
 Copyright (c) 2017-2019 Matthias Geier

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*******************************************************************************/

#include <cassert>
#include <cstring>  // for std::strcmp()
#include <sstream>  // for std::istringstream
#include <fstream>  // for std::ifstream
#include <iterator>  // for std::istreambuf_iterator
#include <optional>

#include <gml/util.hpp>  // for gml::radians()
#include <fmt/format.h>

#include "asdfpp.h"

using namespace asdf;
using namespace fmt::literals;


void ParseError::add_path(const fs::path& path)
{
  _what = "Error loading \"{}\": {}"_format(
      path.native(), _what.size() ? _what : "<no message>");
}

void ParseError::add_xml_data(std::string_view xml_data)
{
  // TODO: make user parameter:
  size_t show_lines_above = 5;

  if (_offset < 0 || xml_data == "")
  {
    return;
  }
  if (static_cast<size_t>(_offset) >= xml_data.size())
  {
    // This should not happen, but who knows ...
    _offset = xml_data.size() - 1;
  }
  size_t error_line = _offset;
  if (error_line > 0)
  {
    --error_line;  // Start searching one character left of offset
  }
  error_line = xml_data.rfind('\n', error_line);
  auto first = error_line;
  for (size_t i = 0
      ; i < show_lines_above && first != std::string::npos
      ; ++i)
  {
    --first;  // Go beyond the previous match
    if (first == std::string::npos) { break; }
    first = xml_data.rfind('\n', first);
  }
  // NB: This relies on: auto x = std::string::npos; ++x; assert(x == 0);
  ++first; ++error_line;  // Don't include \n

  assert(error_line <= static_cast<size_t>(_offset));
  auto marker_pos = _offset - error_line;
  auto marker = std::string(marker_pos, ' ') + "^";
  auto last = xml_data.find('\n', _offset);
  if (last == std::string::npos)
  {
    last = xml_data.size();
  }
  _what = "{}\n---\n{}\n{}"_format(
      _what, xml_data.substr(first, last - first), marker);
}


namespace
{

void accumulate_transforms(const std::optional<Transform>& source
                               , std::optional<Transform>& target)
{
  if (source)
  {
    if (target)
    {
      target->accumulate(*source);
    }
    else
    {
      target = source;
    }
  }
}


void apply_transform(const std::optional<Transform>& source
                         , std::optional<Transform>& target)
{
  if (source)
  {
    if (target)
    {
      target->apply(*source);
    }
    else
    {
      target = source;
    }
  }
}

bool has_name(pugi::xml_node element, const char* name)
{
  return std::strcmp(element.name(), name) == 0;
}

std::optional<vec3> parse_pos(pugi::xml_node element)
{
  auto pos_attr = element.attribute("pos");
  if (pos_attr)
  {
    std::string pos_str = pos_attr.value();
    std::istringstream iss{pos_str};
    float x, y, z{};
    iss >> x >> y;
    iss >> std::ws;
    if (!iss.eof())
    {
      iss >> z;
    }
    iss >> std::ws;
    if (!iss.eof() || iss.fail())
    {
      throw ParseError(
          "Invalid value for \"pos\" attribute: \"{}\""_format(pos_str),
          element, pos_attr);
    }
    return vec3{x, y, z};
  }
  return {};
}

std::optional<quat> parse_rot(pugi::xml_node element)
{
  auto rot_attr = element.attribute("rot");
  if (rot_attr)
  {
    std::string rot_str = rot_attr.value();
    std::istringstream iss{rot_str};
    float azimuth, elevation{}, roll{};
    iss >> azimuth;
    iss >> std::ws;
    if (!iss.eof())
    {
      iss >> elevation;
      iss >> std::ws;
      if (!iss.eof())
      {
        iss >> roll;
        iss >> std::ws;
      }
    }
    if (!iss.eof() || iss.fail())
    {
      throw ParseError(
          "Invalid value for \"rot\" attribute: \"{}\""_format(rot_str),
          element, rot_attr);
    }
    return gml::qrotate(gml::radians(azimuth),   {0.0f, 0.0f, 1.0f})
         * gml::qrotate(gml::radians(elevation), {1.0f, 0.0f, 0.0f})
         * gml::qrotate(gml::radians(roll),      {0.0f, 1.0f, 0.0f});
  }
  return {};
}

}  // end namespace (anonymous)


/// Make sure not to change the scene before throwing this exception.  Don't
/// create IDs, sources, files etc.
struct NoEnd
{
  NoEnd(pugi::xml_node element_, float begin_, ParseError exception_)
  : element{element_}
  , begin{begin_}
  , exception{std::move(exception_)}
  {}

  pugi::xml_node element;
  float begin;
  ParseError exception;
  bool in_seq = false;
};


void Transform::accumulate(const Transform& other)
{
  // NB: Repeated rotation is disallowed
  // TODO: Repeated scaling is disallowed

  if (other.rotation)
  {
    if (this->rotation)
    {
      // TODO: different exception type?
      // TODO: this exception should not terminate the program!
      throw std::runtime_error("Multiple rotations at once");
    }
    else
    {
      // NB: The rotation does not affect this->translation!
      this->rotation = *other.rotation;
    }
  }

  if (other.translation)
  {
    this->apply_translation(*other.translation);
  }

  // TODO: handle other members
}


void Transform::apply(const Transform& other)
{
  // NB: We apply rotation first, then translation.
  if (other.rotation)
  {
    this->apply_rotation(*other.rotation);
  }
  if (other.translation)
  {
    this->apply_translation(*other.translation);
  }
  // TODO: handle other members
}


/// NB: rotation stays unchanged
void Transform::apply_translation(const vec3& other)
{
  if (this->translation)
  {
    *this->translation += other;
  }
  else
  {
    this->translation.emplace(other);
  }
}


/// Rotation (around origin): rotate translation, rotate orientation
void Transform::apply_rotation(const quat& other)
{
  // NB: The order of the following operations doesn't matter
  if (this->translation)
  {
    this->translation = gml::transform(other, *this->translation);
  }
  else
  {
    // Undefined translation stays undefined
  }
  if (this->rotation)
  {
    this->rotation = other * *this->rotation;
  }
  else
  {
    this->rotation = other;
  }
}


Transformer::Transformer(std::string id, float begin, std::optional<float> end
    , const Scene& scene)
: _id(std::move(id))
, _begin_frame(scene.seconds2frames(begin))
, _scene(scene)
{
  if (end)
  {
    _end_frame = _scene.seconds2frames(*end);
  }
}


bool Transformer::overlaps_with(const Transformer& other) const
{
  return (!other._end_frame || _begin_frame < *other._end_frame)
      && (!_end_frame       ||  *_end_frame >  other._begin_frame);
}


std::optional<Transform> Transformer::get_transform(frame_count_t frame) const
{
  // TODO: optimization: store result for last used frame number

  // TODO: check activity before or after memoization?

  std::optional<Transform> result;
  if (_begin_frame <= frame && (!_end_frame || frame < *_end_frame))
  {
    result = this->calculate_transform(frame);
    if (_id != "")
    {
      // TODO: Establish recursion limit! There might be circular dependencies!
      _scene.apply_transforms(_id, frame, result);
    }
  }
  return result;
}


class ConstantTransformer : public Transformer
{
public:
  ConstantTransformer(std::string id, float begin, std::optional<float> end
      , const Scene& scene, Transform transform)
  : Transformer(std::move(id), begin, end, scene)
  , _transform(std::move(transform))
  {}

private:
  Transform calculate_transform(frame_count_t) const override
  {
    return _transform;
  }

  Transform _transform;
};


class SplineTransformer : public Transformer
{
public:
  template<typename C>
  SplineTransformer(std::string id, float begin, std::optional<float> end
      , const Scene& scene, const C& data)
  : Transformer(std::move(id), begin, end, scene)
  , _spline(data)
  {}

private:
  Transform calculate_transform(frame_count_t frame) const override
  {
    Transform result;
    result.translation = _spline.evaluate(_scene.frames2seconds(frame));
    return result;
  }

  Spline _spline;
};


void Source::add_clip_transformer(std::unique_ptr<Transformer> transformer
    , std::string_view source_id, pugi::xml_node element)
{
  for (const auto& existing: _clip_transformers)
  {
    assert(existing);
    if (transformer->overlaps_with(*existing))
    {
      throw ParseError(
          "Source \"{}\": overlapping clips are not allowed"_format(source_id),
          element);
    }
  }
  _clip_transformers.push_back(std::move(transformer));
}


std::optional<Transform> Source::get_clip_transform(frame_count_t frame) const
{
  for (const auto& transformer: _clip_transformers)
  {
    assert(transformer);
    if (auto result = transformer->get_transform(frame))
    {
      // NB: Only one of them can be active, so we don't need to look further
      return result;
    }
  }
  return std::nullopt;
}


Scene::Scene(std::string_view filename, float samplerate
    , FileInfoCallback file_info_callback)
: _path(filename)
, _samplerate(samplerate)
, _file_info_callback(file_info_callback)
{
  std::string xml_data = "";
  try
  {
    if (!fs::exists(_path))
    {
      throw std::runtime_error("File doesn't exist");
    }
    _path = fs::canonical(_path);
    if (fs::is_directory(_path))
    {
      throw std::runtime_error("Given scene file is a directory");
    }
    std::ifstream file(_path);
    xml_data.assign(std::istreambuf_iterator<char>{file}, {});
    _parse_xml_data(xml_data);
  }
  catch (ParseError& e)
  {
    e.add_path(_path);
    e.add_xml_data(xml_data);
    throw e;
  }
  catch (const std::exception& e)
  {
    ParseError new_error{e.what()};
    new_error.add_path(_path);
    throw new_error;
  }
}


void Scene::_parse_xml_data(std::string_view xml_data)
{
  auto options = pugi::parse_default;
  pugi::xml_document doc;
  auto result = doc.load_buffer(xml_data.data(), xml_data.size(), options);

  if (!result)
  {
    throw ParseError(result.description(), result.offset);
  }

  auto asdf = doc.child("asdf");
  if (!asdf)
  {
    throw ParseError("The root element must be <asdf>", doc.first_child());
  }
  auto version_attr = asdf.attribute("version");
  if (!version_attr)
  {
    throw ParseError("'version' attribute is required in <asdf> element", asdf);
  }
  if (std::string(version_attr.value()) != "0.4")
  {
    throw ParseError("Only ASDF version 0.4 is supported", asdf, version_attr);
  }

  // TODO: optional <head> and <body> elements
  auto body = asdf;

  // TODO: <body> cannot have begin/end/dur? Check this?

  float parent_begin = 0;
  std::optional<float> parent_end = std::nullopt;

  // NB: <body> element is implicit <seq>
  auto end = _parse_seq(body.first_child(), parent_begin, parent_end);
  if (end)
  {
    assert(*end >= 0);
    _length = seconds2frames(*end);
  }
  // TODO: check keys in _transformer_map, they must all exist in _source_infos!
}


frame_count_t Scene::seconds2frames(float seconds) const
{
  assert(seconds >= 0);
  assert(_samplerate > 0);
  return static_cast<frame_count_t>(seconds * _samplerate);
}


float Scene::frames2seconds(frame_count_t frames) const
{
  assert(_samplerate > 0);
  return static_cast<float>(frames) / _samplerate;
}


std::optional<float>
Scene::_setup_soundfile(pugi::xml_node element, const fs::path& path
    , float begin, std::optional<float> end)
{
  if (!_file_info_callback)
  {
    throw ParseError(
        "Audio files can only be used if \"file_info_callback\" was specified",
        element);
  }

  auto source_attr = element.attribute("source");
  std::string source_id = _check_source_id(source_attr, element);

  // TODO: check source_id (if non-empty) for source properties
  // TODO: depending on this, the rest may be treated differently
  // NB: only model="point" (default) and model="plane" is allowed for now
  // TODO: allow other source models (e.g. binaural, ambisonics, ...)

  assert(path.is_absolute());
  auto info = _file_info_callback(path, _samplerate);
  auto skip_attr = element.attribute("skip");
  float skip = skip_attr.as_float(0);
  if (skip != 0)
  {
    throw std::runtime_error("TODO: implement 'skip'");
  }
  frame_count_t skip_frames = seconds2frames(skip);
  frame_count_t begin_frame = seconds2frames(begin);
  if (!end)
  {
    if (skip_frames > info.length)
    {
      throw ParseError("\"skip\" attribute cannot be greater than file length"
          , element, skip_attr);
    }
    end = begin + frames2seconds(info.length - skip_frames);
  }
  frame_count_t end_frame = seconds2frames(*end);
  assert(begin_frame <= end_frame);

  PlaylistEntry playlist_entry{};
  playlist_entry.path = path.native();
  // channel_map is set below
  playlist_entry.begin = begin_frame;
  playlist_entry.skip = skip_frames;
  playlist_entry.duration = end_frame - begin_frame;

  std::vector<std::string> channel_ids;
  if (element.first_child())
  {
    if (source_id != "")
    {
      throw ParseError(
          "'source' is only allowed in <clip> if there are no sub-elements",
          element, source_attr);
    }
    // Nothing to do, we continue in loop below ...
  }
  else
  {
    if (info.channels != 1)
    {
      throw ParseError(
          "Multi-channel <clip> must have at least one <channel> sub-element",
          element);
    }
    // NB: A separate channel transform isn't strictly necessary here, but it
    // makes the following code simpler.
    std::string channel_id{};
    Transform transform{};
    _add_channel_to_source(source_id, channel_id, begin, end, transform
        , playlist_entry, channel_ids, element);
  }
  for (auto child: element.children())
  {
    // TODO: <channel> cannot have begin/end/...

    if (!has_name(child, "channel"))
    {
      throw ParseError("Only <channel> elements are allowed in <clip>", child);
    }
    // TODO: allow to skip channels, add -1 to channel_map
    // TODO: use <channel ignore="2"/> or <channel disable="2"/> or ...?
    auto id_attr = child.attribute("id");
    std::string channel_id = _parse_xml_id(id_attr, child);
    auto source_attr = child.attribute("source");
    source_id = _check_source_id(source_attr, child);

    Transform transform{};

    // TODO: get "pos" etc. from channel and add to transform

    _add_channel_to_source(source_id, channel_id, begin, end, transform
        , playlist_entry, channel_ids, child);

    if (playlist_entry.channel_map.size() > info.channels)
    {
      throw ParseError("Only as many <channel> elements as channels in the "
          "file are allowed ({})"_format(info.channels));
    }
  }

  // TODO: require at least one channel?
  assert(channel_ids.size() > 0);
  assert(playlist_entry.channel_map.size() > 0);

  auto id_attr = element.attribute("id");
  // NB: ID may be empty
  std::string clip_id = _parse_xml_id(id_attr, element);
  Transform transform{};

  // TODO: get "pos" etc. from clip and add to transform

  // NB: This transformer applies to channel IDs and not to source IDs!
  _add_transformer(std::make_unique<ConstantTransformer>(
      clip_id, begin, end, *this, transform), channel_ids);

  _playlist.push_back(playlist_entry);
  assert(end);
  return end;
}


/// This creates intentionally invalid XML IDs for internal use
std::string Scene::_create_new_id()
{
  return ".asdf:"_format(_next_internal_id_suffix++);
}


/// https://www.w3.org/TR/xml-id/
///
/// * the ID value matches the allowed lexical form,
/// * the value is unique within the XML document, and that
/// * each element has at most one single unique identifier
///
/// We don't check the last point because pugixml allows multiple attributes
/// with the same name.
///
/// Returning an empty string means the attribute didn't exist.
/// NB: Empty strings are not valid XML IDs!
std::string
Scene::_parse_xml_id(pugi::xml_attribute attribute, pugi::xml_node element)
{
  if (!attribute)
  {
    return "";
  }
  std::string id = attribute.value();
  if (!std::regex_match(id, _re_ncname))
  {
    throw ParseError("Invalid XML ID: \"{}\""_format(id), element, attribute);
  }
  auto [iter_unused, inserted] = _all_ids.insert(id);
  if (!inserted)
  {
    throw ParseError("Non-unique ID: \"{}\""_format(id), element, attribute);
  }
  return id;
}


/// Returning an empty string means the attribute didn't exist.
/// NB: Empty strings are not valid XML IDs!
/// NB: <source id="..."/> can only appear in <head>, <channel source="..." />
/// only in <body>.
std::string
Scene::_check_source_id(pugi::xml_attribute attribute, pugi::xml_node element)
{
  if (!attribute)
  {
    return "";
  }
  std::string id = attribute.value();
  if (id == "")
  {
    throw ParseError("Empty source IDs are not allowed", element, attribute);
  }
  if (_source_exists(id))
  {
    return id;
  }
  id = _parse_xml_id(attribute, element);
  auto& source = _sources.emplace_back();
  source.id = id;
  return id;
}


bool Scene::_source_exists(const std::string& source_id) const
{
  assert(source_id != "");
  auto it = std::find_if(_sources.begin(), _sources.end()
      , [&](const Source& s)
        {
          return s.id == source_id;
        });
  return it != _sources.end();
}


/// Source must exist already!
std::tuple<size_t, Source*> Scene::_get_source(const std::string& source_id)
{
  assert(source_id != "");
  auto it = std::find_if(_sources.begin(), _sources.end()
      , [&](const Source& s)
        {
          return s.id == source_id;
        });
  assert(it != _sources.end());
  auto diff = it - _sources.begin();
  assert(diff >= 0);
  return {diff, &*it};
}


namespace internal
{
  auto get_begin_and_end(pugi::xml_node element
      , float parent_begin, std::optional<float> parent_end)
  {
    // TODO: support hh:mm:ss and similar formats

    // TODO: disallow empty attribute values

    auto attr_begin = element.attribute("begin");
    auto local_begin = attr_begin.as_float(0);
    if (local_begin < 0)
    {
      throw ParseError(
          "Attribute 'begin' must be non-negative", element, attr_begin);
    }
    float begin = parent_begin + local_begin;
    std::optional<float> end;
    auto attr_dur = element.attribute("dur");
    auto attr_end = element.attribute("end");

    if (attr_dur)
    {
      if (attr_end)
      {
        throw ParseError("Attributes 'dur' and 'end' are mutually exclusive"
            , element, attr_end);
      }
      auto dur = attr_dur.as_float();
      if (dur < 0)
      {
        throw ParseError(
            "Attribute 'dur' must be non-negative", element, attr_dur);
      }
      end = begin + dur;
    }
    else if (attr_end)
    {
      end = parent_begin + attr_end.as_float();
      if (end < begin)
      {
        throw ParseError("Attribute 'end' must not be less than 'begin'"
            , element, attr_end);
      }
    }
    else
    {
      end = parent_end;
    }
    if (parent_end)
    {
      if (end)
      {
        end = std::min(*parent_end, *end);
      }
      else
      {
        assert(false);
        end = parent_end;
      }
    }
    return std::pair(begin, end);
  }
}


std::optional<float> Scene::_parse_element(pugi::xml_node element
      , float parent_begin, std::optional<float> parent_end)
{
  if (element.type() != pugi::node_element)
  {
    throw ParseError("Only XML elements are allowed here", element);
  }
  const std::string name{element.name()};
  float begin;
  std::optional<float> end;

  std::tie(begin, end) = internal::get_begin_and_end(
      element, parent_begin, parent_end);
  if (parent_end && begin >= *parent_end)
  {
    throw ParseError(
        "Element would begin after enclosing element has already ended",
        element);
  }
  if (name == "seq")
  {
    // TODO: disallow "id" attribute?
    end = _parse_seq(element.first_child(), begin, end);
  }
  else if (name == "par")
  {
    // TODO: disallow "id" attribute?
    end = _parse_par_children(element, begin, end);
  }
  else if (name == "clip")
  {
    // TODO: allow dummy clip without file?

    auto file_attr = element.attribute("file");
    // TODO: web stream? more things?

    if (file_attr)
    {
      fs::path path{file_attr.value()};
      try
      {
        if (path.empty())
        {
          throw std::runtime_error("Empty file name");
        }
        // NB: File path is relative to scene file
        path = _path.parent_path() / path;
        if (!fs::exists(path))
        {
          throw std::runtime_error("File doesn't exist");
        }
        path = fs::canonical(path);  // This is used for the error message below
        if (fs::is_directory(path))
        {
          throw std::runtime_error("Given file is a directory");
        }
        end = _setup_soundfile(element, path, begin, end);
      }
      catch (const ParseError& e)
      {
        throw;
      }
      catch (const std::exception& e)
      {
        throw ParseError{"Error setting up sound file \"{}\": {}"_format(
            path.native(), *e.what() ? e.what() : "<no message>")
          , element, file_attr};
      }
    }
    else
    {
      throw ParseError("TODO: allow <clip> without \"file\" attribute?"
          , element);
    }
  }
  else if (name == "transform")
  {
    // NB: Parsing of ID must be deferred, because there might be an exception
    auto id_attr = element.attribute("id");

    // NB: multiple targets are allowed
    std::vector<std::string> targets;
    std::istringstream iss{element.attribute("apply-to").value()};
    while (!iss.eof())
    {
      std::string target;
      iss >> target;
      if (iss.fail()) { break; }
      targets.push_back(target);
    }
    if (targets.size() < 1)
    {
      throw ParseError("\"apply-to\" attribute is required for <transform>",
          element);
    }

    std::unique_ptr<Transformer> transformer;

    auto first_child = element.first_child();
    if (!first_child)
    {
      // No sub-elements

      if (auto time_attr = element.attribute("time"))
      {
        throw ParseError("\"time\" attribute is not allowed in <transform>"
            , element, time_attr);
      }

      // TODO: read attributes from <transform>
      // TODO: disallow empty transform with no actual transform in it
      assert(false);
    }
    else if (!first_child.next_sibling())
    {
      // Exactly one sub-element

      if (!has_name(first_child, "p"))
      {
        throw ParseError("Only <p> elements are allowed in <transform>"
            , first_child);
      }
      if (auto time_attr = first_child.attribute("time"))
      {
        throw ParseError("\"time\" attribute is not allowed if there is only "
            "one <p> element", first_child, time_attr);
      }

      Transform t;

      // TODO: combine multiple parsing functions into a helper function?
      t.translation = parse_pos(first_child);
      t.rotation = parse_rot(first_child);
      // TODO: read more attributes

      transformer = std::make_unique<ConstantTransformer>(
          _parse_xml_id(id_attr, element), begin, end, *this, t);
    }
    else
    {
      // More than one sub-element

      // TODO: more lists for other attributes

      std::vector<Spline::AsdfVertex> data;
      for (auto child: element.children())
      {
        if (!has_name(child, "p"))
        {
          throw ParseError("Only <p> elements are allowed in <transform>", child);
        }
        Spline::AsdfVertex vertex;

        // TODO: re-factor into separate functions _parse_vertex, _parse_pos, ...

        // TODO: handle vertices without "pos". If there is any "pos", first and
        // last vertex must also have "pos".

        if (std::strcmp(child.attribute("pos").value(), "closed") == 0)
        {
          vertex.position = CLOSED();
        }
        else
        {
          auto pos = parse_pos(child);
          if (pos)
          {
            vertex.position = *pos;
          }
          else
          {
            // TODO!
          }
        }

        auto time_attr = child.attribute("time");
        if (time_attr)
        {
          // TODO: error checks?
          vertex.time = begin + time_attr.as_float();
        }

        data.push_back(vertex);
      }

      // TODO: proper check
      assert(data.size() != 0);

      // TODO: allow "pos" etc. attributes in <transform> element?

      auto& first_time = data.front().time;

      if (data.size() == 1)
      {
        if (first_time)
        {
          throw ParseError(
              "\"time\" is not allowed if there is only a single vertex",
              element.first_child());
        }

        Transform transform;
        if (std::holds_alternative<vec3>(data.front().position))
        {
          transform.translation = std::get<vec3>(data.front().position);
        }
        else
        {
          throw ParseError(
              "\"closed\" is not allowed if there is only a single vertex",
              element.first_child());
        }

        // TODO: get the other data members

        transformer = std::make_unique<ConstantTransformer>(
            _parse_xml_id(id_attr, element), begin, end, *this, transform);
      }
      else
      {
        if (!first_time)
        {
          first_time = begin;
        }

        auto& last_time = data.back().time;
        if (last_time && !end)
        {
          end = last_time;
        }
        else if (!last_time && end)
        {
          last_time = end;
        }
        if (!last_time)
        {
          throw NoEnd(element, parent_begin
              , {"No \"time\" attribute for last vertex and there is no parent "
                 "object with a specified end/duration", element.last_child()});
        }
        transformer = std::make_unique<SplineTransformer>(
            _parse_xml_id(id_attr, element), begin, end, *this, data);
      }
    }
    _add_transformer(std::move(transformer), targets);
  }
  else
  {
    throw ParseError("Unknown element: <{}>"_format(name), element);
  }
  return end;
}


std::optional<float> Scene::_parse_seq(pugi::xml_node element
    , float parent_begin, std::optional<float> parent_end)
{
  if (!element)
  {
    return parent_end;  // Empty <seq>
  }
  std::optional<float> end;
  try
  {
    end = _parse_element(element, parent_begin, parent_end);
  }
  catch (NoEnd& e)
  {
    e.in_seq = true;
    // NB: There may be multiple nested <seq> elements, none of them is allowed
    // to be followed by another element.
    auto next = element.next_sibling();
    if (next)
    {
      throw ParseError(
          "No element allowed after <{}> with indeterminate duration"_format(
            e.element == element ? element.name() : "seq"), next);
    }
    throw;
  }
  auto next = element.next_sibling();
  if (!next)
  {
    return parent_end ? parent_end : end;  // End of <seq>, end of recursion
  }
  if (!end)
  {
    throw ParseError(
          "No element allowed after <{}> with indeterminate duration"_format(
            element.name()), next);
  }
  return _parse_seq(next, *end, parent_end);
}


std::optional<float> Scene::_parse_par_children(pugi::xml_node parent
      , float parent_begin, std::optional<float> parent_end)
{
  // TODO: option for shortest/longest sibling? (default: longest)

  if (parent_end)
  {
    for (auto child: parent.children())
    {
      _parse_element(child, parent_begin, parent_end);
    }
    return parent_end;
  }

  std::vector<float> ends;
  std::vector<NoEnd> failed;
  for (auto child: parent.children())
  {
    try
    {
      auto end = _parse_element(child, parent_begin, parent_end);
      if (end)
      {
        ends.push_back(*end);
      }
    }
    catch (const NoEnd& e)
    {
      failed.push_back(e);
    }
  }
  auto end = parent_end;
  if (ends.size() > 0)
  {
    end = *std::max_element(ends.begin(), ends.end());
    for (const auto& exception: failed)
    {
      if (exception.in_seq)
      {
        // Continue with partially parsed <seq>
        _parse_seq(exception.element, exception.begin, end);
      }
      else
      {
        assert(exception.begin == parent_begin);
        // "Replay" single element
        _parse_element(exception.element, parent_begin, end);
      }
    }
  }
  else if (failed.size())
  {
    throw failed.front().exception;
  }
  return end;
}


/// Takes ownership of the transformer
template<typename C>
void Scene::_add_transformer(std::unique_ptr<Transformer> transformer
    , const C& targets)
{
  for (const auto& target: targets)
  {
    // Add plain non-owning pointers to map
    // TODO: check if target name is valid?
    _transformer_map[target].push_back(transformer.get());
  }
  // Move owning unique_ptr
  _transformer_storage.push_back(std::move(transformer));
}


void Scene::_add_channel_to_source(const std::string& source_id
    , std::string channel_id, float begin, std::optional<float> end
    , Transform transform, PlaylistEntry& playlist_entry
    , std::vector<std::string>& channel_ids, pugi::xml_node child)
{
  size_t source_number;
  Source* source;
  if (source_id == "")
  {
    // Create a new source with empty ID
    source = &_sources.emplace_back();
    source_number = _sources.size() - 1;
  }
  else
  {
    std::tie(source_number, source) = _get_source(source_id);
  }
  if (channel_id == "")
  {
    // IDs are required for the parent transformer to work
    channel_id = _create_new_id();
  }
  assert(source);
  source->add_clip_transformer(std::make_unique<ConstantTransformer>(
      channel_id, begin, end, *this, transform), source_id, child);
  playlist_entry.channel_map.push_back(source_number);
  channel_ids.push_back(channel_id);
}


std::string Scene::get_source_id(size_t source_number) const
{
  const auto& source = _sources[source_number];
  return source.id;
}


std::string Scene::get_source_name(size_t source_number) const
{
  const auto& source = _sources[source_number];
  return source.name;
}


std::optional<Transform>
Scene::get_source_transform(size_t source_number, frame_count_t frame) const
{
  // NB: This function is supposed to be realtime-safe!

  const auto& source = _sources[source_number];

  auto result = source.get_clip_transform(frame);
  if (!result)
  {
    // NB: If source is not active, we don't need to check other transforms
    return std::nullopt;
  }

  std::optional<Transform> temp;
  this->apply_transforms(source.id, frame, temp);
  accumulate_transforms(temp, result);
  return result;
}


void Scene::apply_transforms(const std::string& target_id
    , frame_count_t frame, std::optional<Transform>& target) const
{
  // NB: This creates an empty list of transformers if key is not available
  // TODO: avoid this?
  const auto& transformers = _transformer_map[target_id];

  // TODO: Establish recursion limit! There might be circular dependencies!

  std::optional<Transform> temp{};
  for (const auto* transformer: transformers)
  {
    assert(transformer);
    accumulate_transforms(transformer->get_transform(frame), temp);
  }
  apply_transform(temp, target);
}
