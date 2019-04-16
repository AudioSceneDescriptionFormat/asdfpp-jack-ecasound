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

#pragma once

#include <unordered_set>

#include <eca-control-interface.h>

#include "asdfpp.h"

namespace asdf {

/// NB: The member functions of this class are not at all thread-safe!
class JackEcasoundScene : public Scene
{
public:
  JackEcasoundScene(std::string_view filename
      , std::string input_port_prefix);

  ~JackEcasoundScene();

  const std::vector<std::string>& get_jack_ports(size_t source_number) const&;

private:
  // Non-const because _portname_stems may be changed
  std::string _get_portname_stem(const std::string& file_path);

  std::string _client_name;
  std::string _input_port_prefix;
  ECA_CONTROL_INTERFACE _eca;
  std::unordered_set<std::string> _portname_stems;
  std::vector<std::vector<std::string>> _jack_ports;
};

}  // namespace asdf
