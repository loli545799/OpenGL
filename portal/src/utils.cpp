#include <iostream>
#include <fstream>
#include <map>
#include "utils.h"


static inline bool isDigital(const char &ch)
{
    return ch >= '0' && ch <= '9';
}

static inline bool isLetter(const char &ch)
{
    return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z');
}

static inline bool isChar(const char &ch)
{
    //return isDigital(ch) || isLetter(ch) || ch == '_' || ch == '.' || ch == '-';
    return !isspace(ch);
}


static inline void errMsg(const unsigned int &lines, const unsigned int &cols, const char &ch, const int &type = 1)
{
    switch (type)
    {
    case 1:
        std::cout << "err: " << lines << ":" << cols << " unknown character " << ch << std::endl;
        break;
    case 2:
        std::cout << "err: " << lines << ":" << cols << " null objname" << std::endl;
        break;
    default:
        std::cout << "err: " << lines << ":" << cols << " unknown character " << ch << std::endl;
        break;
    }
    return;
}

std::vector<std::pair<std::string, std::map<std::string, std::string>>> readConfig(const char *path)
{
    std::vector<std::pair<std::string, std::map<std::string, std::string>>> res;
    std::ifstream config;
    config.open(path);
    if (!config.is_open())
    {
        std::cout << "failed to open file " << path << std::endl;
        return res;
    }
    char ch;
    std::map<std::string, std::string> attr;
    std::string object_str;
    std::string attr_str;
    std::string value_str;
    std::string tmp_str;
    int status = 0;
    int err = 0;
    unsigned int lines = 0, cols = 0;
    config.get(ch);
    while (!config.eof())
    {
        if (ch == '[')
        {
            if (status != 0 && status != 7)
            {
                err = 1;
                break;
            }
            if (!object_str.empty())
            {
                res.push_back(std::pair<std::string, std::map<std::string, std::string>>(object_str, attr));
                object_str.clear();
                attr.clear();
            }
            status = 1;
            tmp_str.clear();
            cols++;
        }
        else if (ch == ']')
        {
            if (status != 2)
            {
                err = 1;
                break;
            }
            if (tmp_str.empty())
            {
                err = 2;
                break;
            }
            status = 3;
            object_str = tmp_str;
            tmp_str.clear();
            cols++;
        }
        else if (ch == '=')
        {
            if (status != 4 && status != 7)
            {
                err = 1;
                break;
            }
            if (tmp_str.empty())
            {
                err = 2;
                break;
            }
            status = 5;
            attr_str = tmp_str;
            tmp_str.clear();
            cols++;
        }
        else if (ch == '\n')
        {
            if (status == 2)
                object_str = tmp_str;
            else if (status == 4)
                attr_str = tmp_str;
            else if (status == 6)
            {
                value_str = tmp_str;
                tmp_str.clear();
                status = 7;
            }
            if (!attr_str.empty() && !value_str.empty())
            {
                attr.insert(std::pair<std::string, std::string>(attr_str, value_str));
                attr_str.clear();
                value_str.clear();
            }
            lines++;
            cols = 0;
        }
        else if (ch == ' ' || ch == '\r' || ch == '\t')
            cols++;
        else if (isChar(ch))
        {
            if (status == 0)
            {
                err = 1;
                break;
            }
            else if (status == 7)
                status == 4;
            else if (status == 1 || status == 3 || status == 5)
                status += 1;
            tmp_str += ch;
            cols++;
        }
        else
        {
            err = 1;
            break;
        }
        config.get(ch);
    }
    config.close();
    if (status == 6)
        value_str = tmp_str;
    else if(status != 0 && status != 7)
        err = 1;
    if (err != 0)
    {
        errMsg(lines, cols, ch, err);
        res.clear();
        return res;
    }
    if (!attr_str.empty() && !value_str.empty())
    {
        attr.insert(std::pair<std::string, std::string>(attr_str, value_str));
        attr_str.clear();
        value_str.clear();
    }
    if (!object_str.empty())
    {
        res.push_back(std::pair<std::string, std::map<std::string, std::string>>(object_str, attr));
        attr.clear();
    }
    return res;
}