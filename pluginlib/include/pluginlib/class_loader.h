/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
#ifndef PLUGINLIB_CLASS_LOADER_H
#define PLUGINLIB_CLASS_LOADER_H

#include "ros/console.h"

#include "pluginlib/class_desc.h"

#include "Poco/ClassLoader.h"
#include "ros/package.h"
#include "tinyxml/tinyxml.h"
#include <vector>
#include <map>

#include "boost/filesystem.hpp"

namespace fs = boost::filesystem;



namespace pluginlib
{

  /**
   * @class ClassLoader
   * @brief A class to help manage and load classes
   */
  template <class T>
    class ClassLoader 
    {
      private:
        typedef std::map<std::string, unsigned int> LibraryCountMap;

      public:
        typedef typename std::map<std::string, ClassDesc>::iterator ClassMapIterator;

      public:
        /**
         * @brief  Constructor for a ClassLoader
         * @param package The package containing the base class
         * @param base_class The type of the base class for classes to be loaded
         * @param attrib_name The attribute to search for in manifext.xml files, defaults to "plugin"
         */
        ClassLoader(std::string package, std::string base_class, std::string attrib_name = std::string("plugin"));

        /**
         * @brief  Destructor for ClassLoader 
         */
        ~ClassLoader();

        /**
         * @brief  Returns a list of all available classes for this ClassLoader's base class type
         * @return A vector of strings corresponding to the names of all available classes
         */
        std::vector<std::string> getDeclaredClasses();

        /**
         * @brief  Given the lookup name of a class, returns the type of the derived class associated with it
         * @param lookup_name The name of the class 
         * @return The name of the associated derived class
         */
        std::string getClassType(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns its description
         * @param lookup_name The lookup name of the class 
         * @return The description of the class
         */
        std::string getClassDescription(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns the type of the associated base class
         * @return The type of the associated base class
         */
        std::string getBaseClassType() const;

        /**
         * @brief  Given the name of a class, returns name of the containing package
         * @param lookup_name The name of the class 
         * @return The name of the containing package
         */
        std::string getClassPackage(const std::string& lookup_name);

        /**
         * @brief  Creates an instance of a desired class, optionally loading the associated library automatically if necessary
         * @param  lookup_name The name of the class to load
         * @param  auto_load Specifies whether or not to automatically load the library containing the class, set to true by default
         * @exception std::runtime_error Thrown when the library cannot be loaded or the class cannot be instantiated 
         * @return An instance of the class
         */
        T* createClassInstance(const std::string& lookup_name, bool auto_load = true);

        /**
         * @brief Checks if a given class is currently loaded
         * @param  lookup_name The lookup name of the class to query
         * @return True if the class is loaded, false otherwise
         */
        bool isClassLoaded(const std::string& lookup_name);

        /**
         * @brief  Attempts to load a class with a given name
         * @param lookup_name The lookup name of the class to load
         * @return True if the class and its associated library were successfully loaded, false otherwise
         */
        bool loadLibraryForClass(const std::string & lookup_name);

      private:
        /**
         * @brief  Given the name of a class, returns the path to its associated library
         * @param lookup_name The name of the class 
         * @return The path to the associated library
         */
        std::string getClassLibraryPath(const std::string& lookup_name);

        /**
         * @brief  Unloads a previously dynamically loaded lobrary
         * @param library_path The library to unload
         * @return True if the library was successfully unloaded, false otherwise
         */
        bool unloadClassLibrary(const std::string& library_path);

        /**
         * @brief  Dynamicaly loads a library
         * @param library_path The library to unload
         * @return True if the library was successfully loaded, false otherwise
         */
        bool loadClassLibrary(const std::string& library_path);

        /**
         * @brief  Returns the names of the classes that are available in a given library
         * @param  library_path The path to the library
         * @return A vector of strings corresponding to the names of the classes in the library
         */
        std::vector<std::string> getClassesInLibrary(const std::string & library_path);

        /**
         * @brief  Returns the libraries that are currently loaded
         * @return A vector of strings corresponding to the names of loaded libraries
         */
        std::vector<std::string> getLoadedLibraries();

        /**
         * @brief  Helper function for loading a shared library
         * @param  library_path The path to the library to load
         * @param  list_name The name of the class list to load
         */
        void loadClassLibraryInternal(const std::string& library_path, const std::string& list_name = std::string(""));

        //used for proper unloading of automatically loaded libraries
        LibraryCountMap loaded_libraries_;

        // map from library to class's descriptions  
        // This is all available classes found in xml
        std::map<std::string, ClassDesc> classes_available_;
        std::string base_class_;

        Poco::ClassLoader<T> poco_class_loader_;  
    };

};

#include "class_loader_imp.h"

#endif //PLUGINLIB_CLASS_LOADER_H
