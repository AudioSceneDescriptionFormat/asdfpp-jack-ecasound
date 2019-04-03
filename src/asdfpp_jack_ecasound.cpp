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

#include <fmt/format.h>
#include <jack/jack.h>  // for jack_port_name_size()

#include "asdfpp_jack_ecasound.h"

using namespace asdf;
using namespace fmt::literals;

// TODO: using "sndfile" made the JACK connection disconnect right after
const static char* AI_OPTIONS = "resample-hq,auto,";

namespace
{

std::string escape_filename(const std::string& filename)
{
  std::string escaped_filename;
  for (const auto ch: filename)
  {
    if (isspace(ch))
    {
      escaped_filename.append(1, '\\');
    }
    escaped_filename.append(1, ch);
  }
  return escaped_filename;
}


void eca_command(ECA_CONTROL_INTERFACE& eca, const std::string& command)
{
  eca.command(command);
  if (eca.error())
  {
    const auto& msg = eca.last_error();
    throw std::runtime_error("Error in ecasound command \"{}\": {}"_format(
          command, msg.size() ? msg : "<no message>"));
  }
}


std::tuple<size_t, float> parse_format_string(std::string_view format_str)
{
  std::string replaced{format_str};
  std::replace(replaced.begin(), replaced.end(), ',', ' ');
  std::istringstream iss(replaced);
  std::string format;
  size_t channels;
  float samplerate;
  iss >> format >> channels >> samplerate;
  if (iss.fail())
  {
    throw std::runtime_error(
        "Couldn't parse format string: \"{}\""_format(format_str));
  }
  return {channels, samplerate};
}


float get_samplerate_from_jack()
{
  ECA_CONTROL_INTERFACE eca;
  eca_command(eca, "ai-add jack");
  eca_command(eca, "ao-add null");
  eca_command(eca, "cs-connect");
  eca_command(eca, "ai-get-format");
  std::string str = eca.last_string();
  eca_command(eca, "cs-disconnect");
  eca_command(eca, "c-remove");
  eca_command(eca, "cs-remove");
  auto [channels, samplerate] = parse_format_string(str);
  assert(samplerate >= 1.0f);
  return samplerate;
}


Scene::FileInfo get_file_info(const std::string& path, float samplerate)
{
  ECA_CONTROL_INTERFACE eca;
  eca_command(eca, "cs-set-audio-format ,,{}"_format(samplerate));
  eca_command(eca, "ai-add {}\"{}\""_format(AI_OPTIONS, escape_filename(path)));
  eca_command(eca, "ao-add null");
  eca_command(eca, "cs-connect");
  eca_command(eca, "ai-index-select 1");
  eca_command(eca, "ai-get-length-samples");
  frame_count_t length = eca.last_long_integer();
  eca_command(eca, "ai-get-format");
  auto [channels, samplerate_control] = parse_format_string(
      eca.last_string());
  assert(samplerate_control == samplerate);
  return {length, channels};
}

}  // end namespace (anonymous)


JackEcasoundScene::JackEcasoundScene(std::string_view filename
    , std::string client_name
    , std::string input_port_prefix)
: Scene(filename, get_samplerate_from_jack(), get_file_info)
, _client_name(std::move(client_name))
, _input_port_prefix(std::move(input_port_prefix))
, _jack_ports(this->number_of_sources())
{
  const auto& playlist = this->get_playlist();

  if (playlist.size())
  {
    eca_command(_eca, "cs-add file-player");

    // TODO: should playlist be ordered?
    for (const auto& file: playlist)
    {
      auto portname_stem = _get_portname_stem(file.path);

      // NB: Each chain must have unique name
      eca_command(_eca, "c-add \"{}\""_format(portname_stem));

      // NB: Set *default* format for following commands:
      eca_command(_eca, "cs-set-audio-format ,{},{}"_format(
            file.channel_map.size(), this->samplerate()));

      // TODO: add loop information
      // TODO: "audioloop" did have no effect together with non-trivial "select"
      // TODO: set total duration of looped playback?
      // TODO: is infinite loop possible? which stop_time?
      eca_command(_eca, "ai-add playat,{}sa,select,{}sa,{}sa,{}\"{}\""_format(
            file.begin, file.skip, file.duration,
            AI_OPTIONS, escape_filename(file.path)));

      eca_command(_eca, "ao-add jack,,\"{}\""_format(portname_stem));

      for (size_t i = 0; i < file.channel_map.size(); ++i)
      {
        _jack_ports.at(file.channel_map[i]).push_back(
            "{}:{}_{}"_format(_client_name, portname_stem, i + 1));
      }
    }
    eca_command(_eca, "cs-option -G:jack,\"{}\",recv"_format(_client_name));
    eca_command(_eca, "cs-connect");
    eca_command(_eca, "engine-launch");
  }
  // TODO: add live inputs (from <head> section) to the JACK ports
  //_jack_ports.at(source_number).push_back(
  //    "{}{}"_format(_input_port_prefix, port_suffix));
}


JackEcasoundScene::~JackEcasoundScene()
{
  // NB: This hopefully never throws!
  _eca.command("cs-disconnect");  // implies "stop" and "engine-halt"
}


const std::vector<std::string>&
JackEcasoundScene::get_jack_ports(size_t source_number) const&
{
  return _jack_ports.at(source_number);
}


std::string JackEcasoundScene::_get_portname_stem(const std::string& file_path)
{
  auto portname_stem{file_path};
  int portname_size = jack_port_name_size();
  portname_size -= _client_name.size();
  --portname_size;  // colon (:)
  portname_size -= 4;  // allow channel suffixes up to _999
  --portname_size;  // terminating \0 character
  assert(portname_size >= 0);
  if (portname_stem.size() > static_cast<size_t>(portname_size))
  {
    portname_stem = portname_stem.substr(portname_stem.size() - portname_size);
    // to visualize the truncation
    portname_stem[0] = '<';
  }
  std::replace(portname_stem.begin(), portname_stem.end(), '/', '_');
  std::replace(portname_stem.begin(), portname_stem.end(), ' ', '_');
  std::replace(portname_stem.begin(), portname_stem.end(), ':', '_');
  std::replace(portname_stem.begin(), portname_stem.end(), '\\', '_');

  int alternative_number = 2;
  std::string unique_portname_stem = portname_stem;
  while (_portname_stems.find(unique_portname_stem) != _portname_stems.end())
  {
    auto suffix = fmt::format("({})", alternative_number++);
    if (suffix.size() >= static_cast<size_t>(portname_size))
    {
      // The file name is provided automatically
      throw std::runtime_error("Unable to find unique JACK port name");
    }
    if (portname_stem.size() + suffix.size()
        <= static_cast<size_t>(portname_size))
    {
      unique_portname_stem = portname_stem + suffix;
    }
    else
    {
      unique_portname_stem = portname_stem.substr(
          (portname_stem.size() + suffix.size()) - portname_size) + suffix;
      unique_portname_stem[0] = '<';
    }
  }
  portname_stem = unique_portname_stem;
  _portname_stems.insert(portname_stem);
  return portname_stem;
}
