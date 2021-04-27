#ifndef OBJPARSER
#define OBJPARSER

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>

class OBJParser
{
public:
    OBJParser();
    ~OBJParser();

    /**
     * Parses the vertices, normals, face vertices and face normals from a file
     * @param fName name of .obj file
     **/
    void parseFile(std::string fName);

    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;

    std::vector<std::vector<int>> faceVertices;
    std::vector<int> faceNormals;

private:
    /**
     * Splits a given string on a given delimiter
     * @param str string to be split
     * @param delim delimiter
     * @return a vector containing the split strings
     **/
    std::vector<std::string> split(std::string &str, char delim);
};

#endif
