#include "OBJParser.h"


OBJParser::OBJParser() {}

void OBJParser::parseFile(std::string fName)
{
    std::string currentLine;
    std::ifstream inFile(fName);
    std::vector<std::string> tokens, indices;

    if (!inFile.is_open())
    {
        std::cout << "Fail to open: " << fName << std::endl;
        return;
    }

    while (std::getline(inFile, currentLine))
    {
        tokens = OBJParser::split(currentLine, ' ');
        if (tokens.size() > 0)
        {
            std::vector<int> faceInd;
            if (tokens[0].compare("v") == 0)
            {
                vertices.push_back(cv::Point3f(std::stof(tokens[1]),std::stof(tokens[3]),(-1 * std::stof(tokens[2]))));
            }
            else if (tokens[0].compare("vn") == 0)
            {
                normals.push_back(cv::Point3f(std::stof(tokens[1]),std::stof(tokens[2]),std::stof(tokens[3])));
            }
            
            else if (tokens[0].compare("f") == 0)
            {
                for(int i = 1 ; i<tokens.size() ; i++)
                {
                    
                    indices = split(tokens[i], '/');
                    faceInd.push_back(std::stoi(indices[0]));
                }
                faceVertices.push_back(faceInd);
                faceInd.clear();
            }
            
        }
    }
}

std::vector<std::string> OBJParser::split(std::string &str, char delim)
{
    std::vector<std::string> tokens;
    std::string part;

    std::stringstream str_stream(str);

    while (std::getline(str_stream, part, delim))
    {
        tokens.push_back(part);
    }

    return tokens;
}

OBJParser::~OBJParser()
{
    vertices.clear();
    normals.clear();
    faceVertices.clear();
    faceNormals.clear();
}
