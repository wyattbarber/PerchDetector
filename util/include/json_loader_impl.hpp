#include <type_traits>
#include <utility>


inline bool _check_key(const Json::Value& root, const std::string& key)
{
    if(!root.isMember(key))
    {
        LOG_JSON_ERR("Key " << key << " not found")
        return false;
    }
    return true;
}


inline bool _check_key(const Json::Value& root, unsigned key)
{
    if(!root.isArray())
    {
        LOG_JSON_ERR("Attempted to access numerical index of non-array object")
        return false;
    }
    if(root.size() <= key)
    {
        LOG_JSON_ERR("Attempted to access out-of-bounds index")
        return false;
    }
    return true;
}


template<typename T, typename K, typename... Ts>
inline bool load_json_value(const Json::Value& root, T& value, const K& key, Ts... Keys)
{
    if(!_check_key(root, key)) return false;
    return load_json_value(root[key], value, Keys...);
}


template<typename T, typename K>
inline bool load_json_value(const Json::Value& root, T& value, const K& key)
{
    if(!_check_key(root, key)) return false;
        

    const Json::Value v = root[key];
    
    if constexpr (std::is_floating_point_v<T>) // Read float type
    {   
        if(!v.isNumeric())
        {
            LOG_JSON_ERR("Value is not numeric.")
            return false;
        }
        value = static_cast<T>(v.asDouble());
    }
    else if constexpr (std::is_integral_v<T> && std::is_unsigned_v<T>) // Read unsigned integer type
    {
        if(!v.isUInt())
        {
            LOG_JSON_ERR("Value is not an unsigned integer.")
            return false;
        }
        value = static_cast<T>(v.asUInt());
    }
    else if constexpr (std::is_integral_v<T>) // Read signed integer value
    {
        if(!v.isInt())
        {
            LOG_JSON_ERR("Value is not an integer.")
            return false;
        }
        value = static_cast<T>(v.asInt());
    }
    else if constexpr (std::is_same_v<T, std::string>) // Read string value
    {
        if(!v.isString())
        {
            LOG_JSON_ERR("Value is not a string.")
            return false;
        }
        value = v.asString();
    }
    else
    {
        // Unhandled type
        LOG_JSON_ERR("Tried to read data to unhandled type: " << typeid(T).name())
        return false;
    }

    return true;
}


template<typename T, typename... Ts>
inline bool load_json_value(const std::string& filename, T& value, Ts... Keys)
{
    std::ifstream file(filename);
    if(!file)
    {
        LOG_JSON_ERR("unable to open " << filename);
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, file, &root, &errs)) {
        LOG_JSON_ERR("Failed to parse " << filename << ": " << errs);
        return false;
    }

    return load_json_value(root, value, Keys...);
}


template<typename K, typename... Ts>
inline bool load_json_object(const Json::Value& root, Json::Value& obj, const K& key, Ts... Keys)
{
    if(!_check_key(root, key)) return false;
    return _load_json_object(root[key], Keys...);
}


template<typename K>
inline bool load_json_object(const Json::Value& root, Json::Value& obj, const K& key)
{
    if(!_check_key(root, key)) return false;
    obj = root[key];
    if(!(obj.isArray() || obj.isObject()))
    {
        LOG_JSON_ERR("Expected array or object but found single value.")
        return false;
    }
    return true;
}


inline bool load_json_object(const Json::Value& root, Json::Value& obj)
{
    obj = root;
    return true;
}


template<typename... Ts>
inline bool load_json_object(const std::string& filename, Json::Value& obj, Ts... Keys)
{
    std::ifstream file(filename);
    if(!file)
    {
        LOG_JSON_ERR("unable to open " << filename);
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, file, &root, &errs)) {
        LOG_JSON_ERR("Failed to parse " << filename << ": " << errs);
        return false;
    }

    return load_json_object(root, obj, Keys...);
}


inline bool load_json_value_pairs(const Json::Value& root)
{
    return true;
}


template<typename T, typename... Ts>
inline bool load_json_value_pairs(const Json::Value& root, const std::string& key, T& value, Ts&&... Pairs)
{
    if(!_check_key(root, key)) return false;
    if(!load_json_value(root, value, key)) return false;
    return load_json_value_pairs(root, std::forward<Ts>(Pairs)...);
}


template<typename T, typename... Ts>
inline bool load_json_value_pairs(const Json::Value& root, int key, T& value, Ts&&... Pairs)
{
    if(!_check_key(root, key)) return false;
    if(!load_json_value(root, value, key)) return false;
    return load_json_value_pairs(root, std::forward<Ts>(Pairs)...);
}


template<typename... Ks, typename... Ts>
inline bool load_json_value_pairs(const std::string& file, const std::tuple<Ks...>& root, Ts&&... Pairs)
{
    Json::Value obj;
    auto get_root = [&file, &obj](auto&&... Keys){ return load_json_object(file, obj, Keys...); };
    
    if(!std::apply(get_root, root)) return false;

    return load_json_value_pairs(obj, std::forward<Ts>(Pairs)...);
}
