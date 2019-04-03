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

/* Audio Scene Description Format */

#pragma once

#include <cstdint>  // for uint64_t
#include <functional>  // for std::function
#include <map>
#include <regex>
#include <unordered_set>
#include <string>
#include <vector>
#include <filesystem>

#include <gml/vec.hpp>  // for gml::vec3
#include <gml/quaternion.hpp>  // for gml::quat
#include <pugixml.hpp>  // for pugi::xml_node and pugi::xml_attribute

#include "asdfspline.hpp"

/// Main ASDF namespace
namespace asdf {

using std::size_t;
using frame_count_t = uint64_t;
using gml::vec3;
using gml::quat;

using Spline = AsdfSpline<float, vec3>;

namespace fs = std::filesystem;

class Scene;

struct Transform
{
  void apply(const Transform& other);
  void accumulate(const Transform& other);

  void apply_translation(const vec3& other);
  void apply_rotation(const quat& other);

  // NB: There must be a well-defined order of operations, i.e.
  //     first scale, then rotation, then translation
  // TODO: scale?
  std::optional<quat> rotation;
  std::optional<vec3> translation;

  // NB: Some operations are order-independent: gain, ...
  // TODO: gain, ...
};

class Transformer
{
public:
  Transformer(std::string id, float begin, std::optional<float> end
      , const Scene& scene);

  bool overlaps_with(const Transformer& other) const;
  std::optional<Transform> get_transform(frame_count_t frame) const;

  virtual ~Transformer() = default;

private:
  virtual Transform calculate_transform(frame_count_t frame) const = 0;

protected:
  std::string _id;
  frame_count_t _begin_frame;
  std::optional<frame_count_t> _end_frame;
  const Scene& _scene;
};

struct Source
{
  std::string id;
  std::string name;

  void add_clip_transformer(std::unique_ptr<Transformer>
      , std::string_view sourcename, pugi::xml_node element);
  std::optional<Transform> get_clip_transform(frame_count_t frame) const;

private:
  /// Those provide the "activity" of the source
  std::vector<std::unique_ptr<Transformer>> _clip_transformers;
};


struct PlaylistEntry
{
  fs::path path;
  /// A 0-based source number for each channel in the file.
  /// Un-assigned channels are marked with -1.
  /// The channel_map can be shorter than the number of channels in the file
  /// (but not longer!).
  std::vector<int> channel_map;
  /// Start (in samples) of playback relative to beginning of scene
  frame_count_t begin;
  /// Number of samples to skip within the file
  frame_count_t skip;
  /// Duration (in samples) of playback
  frame_count_t duration;
};


class ParseError : public std::exception
{
public:
  ParseError(std::string msg, pugi::xml_node node)
  : ParseError(std::move(msg), node.offset_debug())
  {}

  ParseError(std::string msg, pugi::xml_node node
      , pugi::xml_attribute attribute)
  : ParseError(std::move(msg), _estimate_offset(node, attribute))
  {}

  explicit ParseError(std::string msg, ptrdiff_t offset = -1)
  : _what(std::move(msg))
  , _offset(offset)
  {}

  void add_path(const fs::path& path);
  void add_xml_data(std::string_view xml_data);

  const char* what() const noexcept override { return _what.c_str(); }

private:
  static ptrdiff_t
  _estimate_offset(pugi::xml_node node, pugi::xml_attribute attribute)
  {
    // NB: We don't know how many whitespace characters are between attributes
    // (or around the equals signs).  We choose the offset in a way that if
    // minimal whitespace is used, the marker points to the very right of the
    // attribute value.  If more whitespace happens to be used, we still have
    // the full length of the attribute + its value until the marker leaves the
    // correct attribute.  In extreme cases, however, the marker might be too
    // far left.

    auto offset = node.offset_debug();
    if (offset < 0) { return offset; }
    offset += std::strlen(node.name()) - 1;
    offset += std::strlen(node.name()) - 1;
    for (auto attr = attribute; attr; attr = attr.previous_attribute())
    {
      offset += std::strlen(attr.name());
      offset += std::strlen(attr.value());
      offset += 4;  // Equals sign, two quotes, one space before attribute
    }
    return offset;
  }

  std::string _what;
  ptrdiff_t _offset;
};


/// NB: The member functions of this class are not at all thread-safe!
/// pugixml XML conformance: https://pugixml.org/docs/manual.html#loading.w3c
class Scene
{
public:
  struct FileInfo
  {
    frame_count_t length{};  ///< Length of file in samples
    size_t channels{};  ///< Number of channels
  };
  /// Takes an absolute path and a target sampling rate
  /// (sampling rate of the file may be different!), returns a FileInfo struct
  /// with length in samples (based on the given sampling rate) and number of
  /// channels.
  using FileInfoCallback = std::function<FileInfo(const std::string&, float)>;

  /// If file_info_callback is not given, only scenes without audio files can be
  /// loaded.
  /// NB: We assume the samplerate isn't changing during the scene's lifetime
  Scene(std::string_view filename, float samplerate
      , FileInfoCallback file_info_callback = {});

  float samplerate() const { return _samplerate; }
  std::optional<frame_count_t> length() const { return _length; }

  size_t number_of_sources() const { return _sources.size(); }
  std::string get_source_id(size_t source_number) const;
  std::string get_source_name(size_t source_number) const;

  frame_count_t seconds2frames(float seconds) const;
  float frames2seconds(frame_count_t frames) const;

  const std::vector<PlaylistEntry>& get_playlist() const& { return _playlist; };

  /// This is realtime-safe
  /// source_number is 0-based
  std::optional<Transform>
  get_source_transform(size_t source_number, frame_count_t frame) const;

  void apply_transforms(const std::string& target_id, frame_count_t frame
      , std::optional<Transform>& target) const;

private:
  void _parse_xml_data(std::string_view xml_data);
  std::optional<float> _setup_soundfile(pugi::xml_node element
      , const fs::path& path, float begin, std::optional<float> end);
  std::string _create_new_id();
  std::string _parse_xml_id(pugi::xml_attribute attribute
      , pugi::xml_node element);
  std::string _check_source_id(pugi::xml_attribute attribute
      , pugi::xml_node element);
  bool _source_exists(const std::string& source_id) const;
  std::tuple<size_t, Source*> _get_source(const std::string& source_id);

  std::optional<float> _parse_element(pugi::xml_node element
      , float parent_begin, std::optional<float> parent_end);
  std::optional<float> _parse_seq(pugi::xml_node element
      , float parent_begin, std::optional<float> parent_end);
  std::optional<float> _parse_par_children(pugi::xml_node parent
      , float parent_begin, std::optional<float> parent_end);

  template<typename C>
  void _add_transformer(std::unique_ptr<Transformer> transformer
      , const C& targets);
  void _add_channel_to_source(const std::string& source_id
    , std::string channel_id, float begin, std::optional<float> end
    , Transform transform, PlaylistEntry& playlist_entry
    , std::vector<std::string>& channel_ids, pugi::xml_node child);

  size_t _next_internal_id_suffix = 0;
  std::unordered_set<std::string> _all_ids;
  fs::path _path;
  float _samplerate;
  FileInfoCallback _file_info_callback;
  std::optional<frame_count_t> _length;
  std::vector<Source> _sources;
  std::vector<std::unique_ptr<Transformer>> _transformer_storage;
  // NB: mutable because lookup might create new (empty) entry:
  mutable std::map<std::string, std::vector<Transformer*>> _transformer_map;
  std::vector<PlaylistEntry> _playlist;

  // "non-colonized name": https://www.w3.org/TR/xml-names11/#NT-NCName
  std::regex _re_ncname{
    // NCNameStartChar
    "["
    "A-Z_a-z"
    "\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D"
    "\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF"
    "\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\U00010000-\U000EFFFF"
    "]"
    // NCNameChar*
    "["
    "-.0-9\u00B7\u0300-\u036F\u203F-\u2040"
    "A-Z_a-z"
    "\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D"
    "\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF"
    "\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\U00010000-\U000EFFFF"
    "]*"};
};

}  // namespace asdf
