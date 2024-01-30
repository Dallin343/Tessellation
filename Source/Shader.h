//
// Created by Dallin Hagman on 2/28/22.
#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct ShaderData
{
    GLenum type;
    std::string filename;
    int id = -1;
};

class Shader {
public:
    unsigned int ID;
    ShaderData vertex, fragment, tessControl, tessEval;
    // constructor generates the shader on the fly
    // ------------------------------------------------------------------------
    Shader(const char* vertexPath, const char* fragmentPath, const char* tessControlPath = nullptr,
           const char* tessEvalPath = nullptr, const char* geometryPath = nullptr)
    {
        vertex = Compile(GL_VERTEX_SHADER, vertexPath);
        fragment = Compile(GL_FRAGMENT_SHADER, fragmentPath);

        if (tessControlPath != nullptr) {
            tessControl = Compile(GL_TESS_CONTROL_SHADER, tessControlPath);
        }
        if (tessEvalPath != nullptr) {
            tessEval = Compile(GL_TESS_EVALUATION_SHADER, tessEvalPath);
        }

        Link();
//        // 1. retrieve the vertex/fragment source code from filePath
//        std::string vertexCode;
//        std::string fragmentCode;
//        std::string tessControlCode;
//        std::string tessEvaluationCode;
//        std::string geometryCode;
//
//        std::ifstream vShaderFile;
//        std::ifstream fShaderFile;
//        std::ifstream tcShaderFile;
//        std::ifstream teShaderFile;
//        std::ifstream gShaderFile;
//
//        // ensure ifstream objects can throw exceptions:
//        vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//        fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//        tcShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//        teShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//        gShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//        try
//        {
//            // open files
//            vShaderFile.open(vertexPath);
//            fShaderFile.open(fragmentPath);
//            std::stringstream vShaderStream, fShaderStream;
//            // read file's buffer contents into streams
//            vShaderStream << vShaderFile.rdbuf();
//            fShaderStream << fShaderFile.rdbuf();
//            // close file handlers
//            vShaderFile.close();
//            fShaderFile.close();
//            // convert stream into string
//            vertexCode = vShaderStream.str();
//            fragmentCode = fShaderStream.str();
//            // if geometry shader path is present, also load a geometry shader
//            if (tessControlPath != nullptr)
//            {
//                tcShaderFile.open(tessControlPath);
//                std::stringstream tcShaderStream;
//                tcShaderStream << tcShaderFile.rdbuf();
//                tcShaderFile.close();
//                tessControlCode = tcShaderStream.str();
//            }
//            if (tessEvalPath != nullptr)
//            {
//                teShaderFile.open(tessEvalPath);
//                std::stringstream teShaderStream;
//                teShaderStream << teShaderFile.rdbuf();
//                teShaderFile.close();
//                tessEvaluationCode = teShaderStream.str();
//            }
//            if(geometryPath != nullptr)
//            {
//                gShaderFile.open(geometryPath);
//                std::stringstream gShaderStream;
//                gShaderStream << gShaderFile.rdbuf();
//                gShaderFile.close();
//                geometryCode = gShaderStream.str();
//            }
//        }
//        catch (std::ifstream::failure& e)
//        {
//            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
//        }
//        const char* vShaderCode = vertexCode.c_str();
//        const char * fShaderCode = fragmentCode.c_str();
//        // 2. compile shaders
//        unsigned int vertex, fragment;
//        // vertex shader
//        vertex = glCreateShader(GL_VERTEX_SHADER);
//        glShaderSource(vertex, 1, &vShaderCode, nullptr);
//        glCompileShader(vertex);
//        checkCompileErrors(vertex, "VERTEX");
//        // fragment Shader
//        fragment = glCreateShader(GL_FRAGMENT_SHADER);
//        glShaderSource(fragment, 1, &fShaderCode, nullptr);
//        glCompileShader(fragment);
//        checkCompileErrors(fragment, "FRAGMENT");
//
//        unsigned int tessControl, tessEval;
//        if (tessControlPath != nullptr) {
//            const char * tcShaderCode = tessControlCode.c_str();
//            tessControl = glCreateShader(GL_TESS_CONTROL_SHADER);
//            glShaderSource(tessControl, 1, &tcShaderCode, nullptr);
//            glCompileShader(tessControl);
//            checkCompileErrors(tessControl, "TESSELLATION CONTROL");
//        }
//
//        if (tessEvalPath != nullptr) {
//            const char * teShaderCode = tessEvaluationCode.c_str();
//            tessEval = glCreateShader(GL_TESS_EVALUATION_SHADER);
//            glShaderSource(tessEval, 1, &teShaderCode, nullptr);
//            glCompileShader(tessEval);
//            checkCompileErrors(tessEval, "TESSELLATION EVALUATION");
//        }
//
//        // if geometry shader is given, compile geometry shader
//        unsigned int geometry;
//        if(geometryPath != nullptr)
//        {
//            const char * gShaderCode = geometryCode.c_str();
//            geometry = glCreateShader(GL_GEOMETRY_SHADER);
//            glShaderSource(geometry, 1, &gShaderCode, nullptr);
//            glCompileShader(geometry);
//            checkCompileErrors(geometry, "GEOMETRY");
//        }
//        // shader Program
//        ID = glCreateProgram();
//        glAttachShader(ID, vertex);
//        glAttachShader(ID, fragment);
//        if (tessControlPath != nullptr)
//        {
//            glAttachShader(ID, tessControl);
//        }
//        if (tessEvalPath != nullptr)
//        {
//            glAttachShader(ID, tessEval);
//        }
//        if(geometryPath != nullptr)
//        {
//            glAttachShader(ID, geometry);
//        }
//
//        glLinkProgram(ID);
//        checkCompileErrors(ID, "PROGRAM");
//        // delete the shaders as they're linked into our program now and no longer necessery
//        glDeleteShader(vertex);
//        glDeleteShader(fragment);
//        if (tessControlPath != nullptr)
//        {
//            glDeleteShader(tessControl);
//        }
//        if (tessEvalPath != nullptr)
//        {
//            glDeleteShader(tessEval);
//        }
//        if(geometryPath != nullptr)
//        {
//            glDeleteShader(geometry);
//        }
    }
    // activate the shader
    // ------------------------------------------------------------------------
    void use() const
    {
        glUseProgram(ID);
    }

    std::string typeToStr(GLenum type) {
        switch (type) {
            case GL_VERTEX_SHADER: return "VERTEX";
            case GL_FRAGMENT_SHADER: return "FRAGMENT";
            case GL_TESS_CONTROL_SHADER: return "TESSELLATION_CONTROL";
            case GL_TESS_EVALUATION_SHADER: return "TESSELLATION_EVALUATION";
            default: return "UNKNOWN";
        }
    }

    void Link() {
        ID = glCreateProgram();
        glAttachShader(ID, vertex.id);
        glAttachShader(ID, fragment.id);
        if (tessControl.id != -1) {
            glAttachShader(ID, tessControl.id);
        }
        if (tessEval.id != -1) {
            glAttachShader(ID, tessEval.id);
        }

        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no longer necessery
        glDeleteShader(vertex.id);
        glDeleteShader(fragment.id);
        if (tessControl.id != -1)
        {
            glDeleteShader(tessControl.id);
        }
        if (tessEval.id != -1)
        {
            glDeleteShader(tessEval.id);
        }

    }

    void Reload() {
        glDeleteProgram(ID);

        vertex = Compile(GL_VERTEX_SHADER, vertex.filename);
        fragment = Compile(GL_FRAGMENT_SHADER, fragment.filename);

        if (tessControl.id != -1) {
            tessControl = Compile(GL_TESS_CONTROL_SHADER, tessControl.filename);
        }
        if (tessEval.id != -1) {
            tessEval = Compile(GL_TESS_EVALUATION_SHADER, tessEval.filename);
        }

        Link();
    }

    ShaderData Compile(GLenum type, const std::string& filename) {
        ShaderData result;
        result.type = type;
        result.filename = filename;
        std::string code;

        std::ifstream shaderFile;

        // ensure ifstream objects can throw exceptions:
        shaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        try
        {
            // open files
            shaderFile.open(filename);
            std::stringstream shaderStream;
            // read file's buffer contents into streams
            shaderStream << shaderFile.rdbuf();
            // close file handlers
            shaderFile.close();
            // convert stream into string
            code = shaderStream.str();
        }
        catch (std::ifstream::failure& e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
        }

        const char* shaderCode = code.c_str();
        // 2. compile shaders
        result.id = glCreateShader(type);
        glShaderSource(result.id, 1, &shaderCode, nullptr);
        glCompileShader(result.id);
        checkCompileErrors(result.id, typeToStr(type));

        return result;
    }

    // utility uniform functions
    // ------------------------------------------------------------------------
    void setBool(const std::string &name, bool value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
    }
    // ------------------------------------------------------------------------
    void setInt(const std::string &name, int value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }
    void setIntArr(const std::string &name, int* value, int count) const
    {
        glUniform1iv(glGetUniformLocation(ID, name.c_str()), count, value);
    }
    // ------------------------------------------------------------------------
    void setFloat(const std::string &name, float value) const
    {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }
    void setFloatArr(const std::string &name, float* value, int count) const
    {
        glUniform1fv(glGetUniformLocation(ID, name.c_str()), count, value);
    }
    // ------------------------------------------------------------------------
    void setVec2(const std::string &name, const glm::vec2 &value) const
    {
        glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec2(const std::string &name, float x, float y) const
    {
        glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
    }
    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const
    {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec3(const std::string &name, float x, float y, float z) const
    {
        glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const
    {
        glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec4(const std::string &name, float x, float y, float z, float w) const
    {
        glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::ivec4 &value) const
    {
        glUniform4iv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec4(const std::string &name, int x, int y, int z, int w) const
    {
        glUniform4i(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }
    void setVec4Arr(const std::string &name, const glm::ivec4* value, int count) const
    {
        glUniform4iv(glGetUniformLocation(ID, name.c_str()), count, glm::value_ptr(value[0]));
    }
    // ------------------------------------------------------------------------
    void setMat2(const std::string &name, const glm::mat2 &mat) const
    {
        glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat3(const std::string &name, const glm::mat3 &mat) const
    {
        glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat4(const std::string &name, const glm::mat4 &mat) const
    {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }

private:
    // utility function for checking shader compilation/linking errors.
    // ------------------------------------------------------------------------
    static void checkCompileErrors(GLuint shader, const std::string& type)
    {
        GLint success;
        GLchar infoLog[1024];
        if(type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if(!success)
            {
                glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if(!success)
            {
                glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};
#endif
