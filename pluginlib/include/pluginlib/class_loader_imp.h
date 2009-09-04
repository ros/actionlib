/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

//NOTE: this should really never be included on its own, but just in case someone is bad we'll guard

#ifndef PLUGINLIB_CLASS_LOADER_IMP_H_
#define PLUGINLIB_CLASS_LOADER_IMP_H_

#include <stdexcept>

namespace pluginlib {
  template <class T>
  ClassLoader<T>::ClassLoader(std::string package, std::string base_class, std::string attrib_name) : base_class_(base_class)
  {
    //Pull possible files from manifests of packages which depend on this package and export class
    std::vector<std::string> paths;
    ros::package::getPlugins(package, attrib_name, paths);

    //The poco factory for base class T
    for (std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it)
    {
      TiXmlDocument document;
      document.LoadFile(*it);
      TiXmlElement * config = document.RootElement();
      if (config == NULL)
      {
        ROS_ERROR("XML Document \"%s\" had no Root Element.  This likely means the XML is malformed or missing.", it->c_str());
        return;
      }
      if (config->ValueStr() != "library" &&
          config->ValueStr() != "class_libraries")
      {
        ROS_ERROR("The XML given to add must have either \"library\" or \
            \"class_libraries\" as the root tag");
        return ;
      }
      //Step into the filter list if necessary
      if (config->ValueStr() == "class_libraries")
      {
        config = config->FirstChildElement("library");
      }

      TiXmlElement* library = config;
      while ( library != NULL)
      {
        std::string library_path = library->Attribute("path");
        if (library_path.size() == 0)
        {
          ROS_ERROR("Failed to find Path Attirbute in library element in %s", it->c_str());
          continue;
        }

        std::string package_name;

        fs::path p(*it);
        fs::path parent = p.parent_path();
        // figure out the package this class is part of
        while (true)
        {
          if (fs::exists(parent / "manifest.xml"))
          {
            std::string package = parent.filename();
            std::string package_path = ros::package::getPath(package);
            if (it->find(package_path) == 0)
            {
              package_name = package;
              break;
            }
          }

          parent = parent.parent_path();

          if (parent.string().empty())
          {
            ROS_ERROR("Could not find package name for class %s", it->c_str());
            break;
          }
        }
        fs::path full_library_path(parent / library_path);

        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element)
        {
          std::string base_class_type = class_element->Attribute("base_class_type");
          std::string lookup_name = class_element->Attribute("name");
          std::string derived_class = class_element->Attribute("type");

          
          
          //make sure that this class is of the right type before registering it
          if(base_class_type == base_class){

            // register class here
            TiXmlElement* description = class_element->FirstChildElement("description");
            std::string description_str = description ? description->GetText() : "";

            classes_available_.insert(std::pair<std::string, ClassDesc>(lookup_name, ClassDesc(lookup_name, derived_class, base_class_type, package_name, description_str, full_library_path.string())));
            ROS_DEBUG("MATCHED Base type for class with name: %s type: %s base_class_type: %s Expecting base_class_type %s", 
                      lookup_name.c_str(), derived_class.c_str(), base_class_type.c_str(), base_class.c_str());
          }
          else
          {
            ROS_DEBUG("UNMATCHED Base type for class with name: %s type: %s base_class_type: %s Expecting base_class_type %s", 
                      lookup_name.c_str(), derived_class.c_str(), base_class_type.c_str(), base_class.c_str());
            
          }
          //step to next class_element
          class_element = class_element->NextSiblingElement( "class" );
        }
        library = library->NextSiblingElement( "library" );
      }
    }
  }

  template <class T>
  bool ClassLoader<T>::loadLibraryForClass(const std::string & lookup_name)
  {
    std::string library_path;
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end()){
      library_path = it->second.library_path_;
    }
    else
    {
      //\todo make this a function call
      std::string declared_types;
      std::vector<std::string> types = getDeclaredClasses();
      for ( unsigned int i = 0; i < types.size(); i ++)
      {
        declared_types = declared_types + std::string(" ") + types[i];
      }
      ROS_ERROR("According to the loaded plugin descriptions the class %s with base class type %s does not exist.  Declared types are %s", 
                lookup_name.c_str(), base_class_.c_str(), declared_types.c_str() );
      return false;
    }
    library_path.append(Poco::SharedLibrary::suffix());
    try
    {
      ROS_DEBUG("Attempting to load library %s for class %s",
                library_path.c_str(), lookup_name.c_str());
      
      loadClassLibraryInternal(library_path, lookup_name);
    }
    catch (Poco::LibraryLoadException &ex)
    {
      ROS_ERROR("Failed to load library %s Error string: %s", library_path.c_str(), ex.displayText().c_str());
      return false;
    }
    catch (Poco::NotFoundException &ex)
    {
      ROS_ERROR("Failed to find library %s Error string: %s", library_path.c_str(), ex.displayText().c_str());
      return false;
    }
    return true;
  }

  template <class T>
  ClassLoader<T>::~ClassLoader()
  {
    for (LibraryCountMap::iterator it = loaded_libraries_.begin(); it != loaded_libraries_.end(); ++it)
    {
      if ( it->second > 0)
        unloadClassLibrary(it->first);
    }
  }


  template <class T>
  bool ClassLoader<T>::isClassLoaded(const std::string& lookup_name)
  {
    try
    {
      return poco_class_loader_.canCreate(getClassType(lookup_name));
    }
    catch (Poco::RuntimeException &ex)
    {
      return false;
    }
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getDeclaredClasses()
  {
    std::vector<std::string> lookup_names;
    for (ClassMapIterator it = classes_available_.begin(); it != classes_available_.end(); ++it)
    {
      lookup_names.push_back(it->first);
    }
    return lookup_names;
  }

  template <class T>
  std::string ClassLoader<T>::getClassType(const std::string& lookup_name)
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.derived_class_;
    return "";
  }

  template <class T>
  std::string ClassLoader<T>::getClassDescription(const std::string& lookup_name)
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.description_;
    return "";
  }

  template <class T>
  std::string ClassLoader<T>::getBaseClassType() const
  {
    return base_class_;
  }

  template <class T>
  std::string ClassLoader<T>::getClassLibraryPath(const std::string& lookup_name)
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.library_path_;
    return "";
  }

  template <class T>
  std::string ClassLoader<T>::getClassPackage(const std::string& lookup_name)
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.package_;
    return "";
  }

  template <class T>
  T* ClassLoader<T>::createClassInstance(const std::string& lookup_name, bool auto_load)
  {
    if ( auto_load && !isClassLoaded(lookup_name))
      if(!loadLibraryForClass(lookup_name))
      {
        ROS_ERROR("Failed to auto load library");
        throw std::runtime_error("Failed to auto load library for class " + lookup_name + ".");
      }

    try{
      return poco_class_loader_.create(getClassType(lookup_name));
    }
    catch(const Poco::RuntimeException& ex){
      ROS_ERROR("Poco exception: %s (class: %s)", ex.displayText().c_str(), lookup_name.c_str());
      throw std::runtime_error(ex.what());
    }
  }

  template <class T>
  bool ClassLoader<T>::unloadClassLibrary(const std::string& library_path)
  {
    LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
    if (it == loaded_libraries_.end())
    {
      ROS_DEBUG("unable to unload library which is not loaded");
      return false;
    }
    else if (it-> second > 1)
      (it->second)--;
    else
      poco_class_loader_.unloadLibrary(library_path);

    return true;

  }

  template <class T>
  bool ClassLoader<T>::loadClassLibrary(const std::string& library_path){
    try
    {
      loadClassLibraryInternal(library_path);
    }
    catch (Poco::LibraryLoadException &ex)
    {
      return false;
    }
    catch (Poco::NotFoundException &ex)
    {
      return false;
    }
    return true;
  }

  template <class T>
  void ClassLoader<T>::loadClassLibraryInternal(const std::string& library_path, const std::string& list_name) {
    poco_class_loader_.loadLibrary(library_path, list_name);
    LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
    if (it == loaded_libraries_.end())
      loaded_libraries_[library_path] = 1;  //for correct destruction and access
    else
      loaded_libraries_[library_path] = loaded_libraries_[library_path] + 1;
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getClassesInLibrary(const std::string & library_path)
  {
    std::vector<std::string> lookup_names;


    const Poco::Manifest<T> * manifest = poco_class_loader_.findManifest(library_path);
    if (manifest == NULL)
      return lookup_names;

    for (typename Poco::Manifest<T>::Iterator it = manifest->begin(); it != manifest->end(); ++it)
    {
      lookup_names.push_back(it->name());
    }
    return lookup_names;
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getLoadedLibraries()
  {
    std::vector<std::string> library_names;

    /*
       \todo find a way to get ths out of poco
       for (typename Poco::ClassLoader<T>::Iterator it = poco_class_loader_.begin(); it != poco_class_loader_.end(); ++it)
       {
       library_names.push_back(it->second->className());
       }
       return library_names;
       */
    LibraryCountMap::iterator it;
    for (it = loaded_libraries_.begin(); it != loaded_libraries_.end(); it++)
    {
      if (it->second > 0)
        library_names.push_back(it->first);
    }
    return library_names;
  }
};

#endif
