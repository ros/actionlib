/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef PLUGINLIB_BOOST_FS_WRAPPER_H
#define PLUGINLIB_BOOST_FS_WRAPPER_H


#include "pluginlib/boost_fs_wrapper.h"
#include "boost/filesystem.hpp"
#include "ros/package.h"

namespace pluginlib
{

std::string joinPaths(const std::string& path1, const std::string& path2)
{
  boost::filesystem::path p1(path1);
  return (p1 / path2).string();
}

std::string getPackageFromLibraryPath(const std::string & path)
{

  std::string package_name;

  boost::filesystem::path p(path);
  boost::filesystem::path parent = p.parent_path();
  // figure out the package this class is part of
  while (true)
  {
    if (boost::filesystem::exists(parent / "manifest.xml"))
    {
#if BOOST_FILESYSTEM_VERSION && BOOST_FILESYSTEM_VERSION == 3
      std::string package = parent.filename().string();
#else
      std::string package = parent.filename();
#endif
      std::string package_path = ros::package::getPath(package);
      if (path.find(package_path) == 0)
      {
        package_name = package;
        break;
      }
    }

#if BOOST_FILESYSTEM_VERSION && BOOST_FILESYSTEM_VERSION == 3
    parent = parent.parent_path().string();
#else
    parent = parent.parent_path();
#endif

    if (parent.string().empty())
    {
      return "";
    }
  }

  return package_name;
}


}


#endif //PLUGINLIB_BOOST_FS_WRAPPER_H
