#pragma once

#include <opencv2/core/mat.hpp>
#include "json/json.h"
#include <fstream>
#include "Logging.hpp"
#include <iostream>


#define LOG_JSON_ERR(content) Logger::instance() << "[ERROR][_json_loader_util] " << content << std::endl;


/** Loads a single value from a json file.

The given value for Keys may be a single string or int,
or sequence of strings and ints. Strings are treated as keys 
to a mapping, while ints are treated as zero-based indices in an 
array. The sequence of keys is followed starting at the file root,
and the value of the final key is provided. 

@param filename Name of the file to read data from.
@param value Reference to the value to set with data from the file.
@param Keys sequence of keys to reach the object
@tparam T scalar datatype of the item to find
*/
template<typename T, typename... Ts>
bool load_json_value(const std::string& filename, T& value, Ts... Keys);

/** Helper overload to load an item from a parsed object.*/
template<typename T, typename... Ts>
bool load_json_value(const Json::Value& root, T& value, Ts... Keys);

/** Load camera calibration parameters from a file.

Expects the file to be a json file containing the following objects:

*    distortion: A list of floats containing 5 elements

*    intrinsic: A list of floats containing 9 elements of a matrix. 
        Expects the list to contain all elements of the first row, 
        followed by all elements of the second row, then the third.

@param filename Fame of the file to read from
@param dist 1x5 distortion coefficient matrix
@param intr 3x3 intrinsic coefficient matrix

@return true if data loaded succesfully
*/
bool load_camera_matrices(const std::string& filename, cv::Mat& dist, cv::Mat& intr);


/** Load stereo camera parameters from a file.

Expects the file to be a json file containing the following objects:

*    translation: A list of floats containing 3 elements

*    rotation: A list of floats containing 9 elements of a matrix. 
        Expects the list to contain all elements of the first row, 
        followed by all elements of the second row, then the third.

@param filename Fame of the file to read from
@param R 3x3 rotation matrix
@param T 3x1 translation matrix

@return true if data loaded succesfully
*/
bool load_stereo_matrices(const std::string& filename, cv::Mat& R, cv::Mat& T);


/** Load a JSON object or array.

@param filename Name of the file to read from.
@param obj JSON object to populate with data.
@param Keys Sequence (possibly empty) of keys to reach the object.

@return true if the object was found.
*/
template<typename... Ts>
bool load_json_object(const std::string& filename, Json::Value& obj, Ts... Keys);

/** Overloaded helper to read an object from an opened file. */
template<typename... Ts>
bool load_json_object(const Json::Value& root, Json::Value& obj, Ts... Keys);


/** Maps a set of keys to data values of different types.

The parameter pack arguments should be an even number of pairs,
where the first argument of each pair is a string or integer used
to key the given root object, and the second is a reference to the 
variable that the data in the given key will be loaded to.

For example `bool ok = load_json_value_pairs(root, "float_val", float_var_ref, "int_val", int_var_ref);`

@param root Root object to read data from.
@param Pairs Sequence of key-reference pairs to read and map data.

@return true if all values were loaded successfully.
*/
template<typename T, typename... Ts>
bool load_json_value_pairs(const Json::Value& root, const std::string& key, T& value, Ts&&... Pairs);
template<typename T, typename... Ts>
bool load_json_value_pairs(const Json::Value& root, int key, T& value, Ts&&... Pairs);

/** Overloaded helper to load items from an unopened file and a path to the root of the items.

@param file Name of the file to open
@param root Path (as tuple of keys) to the root of the parameters to load.
@param Pairs Sequence of key-reference pairs to read and map data.

@return true if all values were loaded successfully.
*/
template<typename... Ks, typename... Ts>
bool load_json_value_pairs(const std::string& file, const std::tuple<Ks...>& root, Ts&&... Pairs);

#include "json_loader_impl.hpp"