#include "metahubmemorysharing.h"

#include"string.h"
#include "yaml-cpp/yaml.h"
#include<QByteArray>
#include <fstream>
#include<QFile>
#include<QDir>
#include<QDebug>
#include "cloudcompareutils/icore.h"
using namespace YAML;

#define INPUT_SIZE  18
#define PROCESS_NAME  L"Metahubfjslam"  

void CS::RosMemory::RosMemorySharing::slamGetyamlData(std::string yamlpath)
{
    std::string mapconfig_yamlPath = yamlpath + "mapconfig.yaml";
    std::string fjslam_yamlPath = yamlpath ;
    YAML::Node mapconfig_yaml = YAML::LoadFile(mapconfig_yamlPath);
    std::wstring project_name = UtfToGbk(mapconfig_yaml["project_name"].as<std::string>());
    std::wstring project_savePath = UtfToGbk(mapconfig_yaml["project_savePath"].as<std::string>());
    std::wstring project_tempPath = UtfToGbk(mapconfig_yaml["project_tempPath"].as<std::string>());
   int slam_mode = mapconfig_yaml["slam_mode"].as<int>();
   float ignore_startTime = mapconfig_yaml["ignore_startTime"].as<float>();
   float ignore_endTime = mapconfig_yaml["ignore_endTime"].as<float>();
   int total_pcdNum = mapconfig_yaml["total_pcdNum"].as<int>();
}
std::wstring CS::RosMemory::RosMemorySharing::UtfToGbk(std::string strValue)
{
    int num = MultiByteToWideChar(CP_UTF8, 0, strValue.c_str(), -1, NULL, 0);
    wchar_t *wide = new wchar_t[num];
    MultiByteToWideChar(CP_UTF8, 0, strValue.c_str(), -1, wide, num);
    std::wstring w_str(wide);
    delete[] wide;
    return w_str;
}
void CS::RosMemory::RosMemorySharing::removeTrailingNewline(std::string& str) {
    if (!str.empty() && str[str.length() - 1] == '\n') {
        str.pop_back();
    }
}
bool CS::RosMemory::RosMemorySharing::createMainYamlFiles(QMap<QString, QString>map)
{
    //[!]name
    QByteArray Array = map.value("NAME").toUtf8();
    std::string projectName = Array.data();
    removeTrailingNewline(projectName);
    Array.clear();
    //[!]save path
    Array = QDir::toNativeSeparators(map.value("SAVE")).toUtf8() + "\\";
    std::string projectSavePath = Array.data();
    removeTrailingNewline(projectSavePath);
    Array.clear();
    //[!]temp Path
    Array = QDir::toNativeSeparators(map.value("TEMPPATH")).toUtf8() + "\\";
    std::string tempPath = Array.data();
    removeTrailingNewline(tempPath);
    Array.clear();
    //[!]Correctionfilepath
    Array = QDir::toNativeSeparators(map.value("correction.csv")).toUtf8();
    std::string algorithm_Correction = Array.data();
    removeTrailingNewline(algorithm_Correction);
    Array.clear();
    //[!]FJD_Trion_S1
    Array = QDir::toNativeSeparators(map.value("FJD_Trion_S1.yaml")).toUtf8();
    std::string algorithm_FJD_Trion_S1 = Array.data();
    removeTrailingNewline(algorithm_FJD_Trion_S1);
    Array.clear();
    //[!]FJD_Trion_P1
    Array = QDir::toNativeSeparators(map.value("FJD_Trion_P1.yaml")).toUtf8();
    std::string algorithm_FJD_Trion_P1 = Array.data();
    removeTrailingNewline(algorithm_FJD_Trion_P1);
    Array.clear();
    //[!]FJDTrionP1FjSlamPath
    Array = QDir::toNativeSeparators(map.value("FJDTrionP1FjSlamPath")).toUtf8();
    std::string FJDTrionP1FjSlamPath = Array.data();
    removeTrailingNewline(FJDTrionP1FjSlamPath);
    Array.clear();
    //[!]controlFilePath
    std::string controlFilePath = map.value("ControlFilePath").toUtf8().data();
    removeTrailingNewline(controlFilePath);
    Array.clear();

    YAML::Node write_fjslam_params_yaml;
    write_fjslam_params_yaml["product_type"] = map.value("product_type").toInt();						// product_type: 1 # s1
    write_fjslam_params_yaml["MappingModel"] = map.value("MappingModel").toInt();						//[!]MappingModel
    write_fjslam_params_yaml["project_name"] = projectName;												//工程名称
    write_fjslam_params_yaml["project_savePath"] = projectSavePath;										//文件保存路径
    write_fjslam_params_yaml["project_tempPath"] = tempPath;											//[!]工作地址
    write_fjslam_params_yaml["correctionfilepath"] = algorithm_Correction;								//[!]correction文件路径
    write_fjslam_params_yaml["FJDTrionP1ProfilePath"] =
    static_cast<bool>(map.value("product_type").toInt()) ? algorithm_FJD_Trion_S1 : algorithm_FJD_Trion_P1;				//[!] yaml
    write_fjslam_params_yaml["FJDTrionP1FjSlamPath"] = FJDTrionP1FjSlamPath;							//[!] fjslam
    write_fjslam_params_yaml["ControlEnableFlag"] =static_cast<bool>(map.value("ControlEnableFlag").toInt());				//[!]ControlEnableFlag
    if (map.value("ControlEnableFlag").toInt()) {
        write_fjslam_params_yaml["ControlFilePath"] = controlFilePath;		//[!]ControlFilePath
    }
    write_fjslam_params_yaml["gpsEnableFlag"] = map.value("gpsEnableFlag").toInt();
    //[!]建图范围
    write_fjslam_params_yaml["reconstruction_scope_min"] = QString::number(map.value("reconstruction_scope_min").toFloat(), 'f', 2).toUtf8().data();
    write_fjslam_params_yaml["reconstruction_scope_max"] = QString::number(map.value("reconstruction_scope_max").toFloat(), 'f', 2).toUtf8().data();

    YAML::Emitter emitter;
    emitter << write_fjslam_params_yaml;
    std::ofstream output_file(map.value("mapconfig.yaml").toStdWString());
    if (output_file.is_open()) {
        output_file << emitter.c_str();
        output_file.close();
        std::string yamlData = emitter.c_str();
        QByteArray byteArray = QByteArray::fromStdString(yamlData);
        qDebug() << "mappingConfigFile:" << QString::fromUtf8(byteArray);
    }
    else {
        return false;
    }

    if (map.value("product_type").toInt())
    {
        std::wstring string_S1FilePath = map.value("FJD_Trion_S1.yaml").toStdWString();
        std::ifstream fin(string_S1FilePath);
        if (!fin.is_open()) {
            std::cerr << "Failed to open file!\n";
            return false;
        }
        YAML::Node root = YAML::Load(fin);
        if (root["mapping"]) {
            root["mapping"]["s1_map_save_min_dis"] = QString::number(map.value("reconstruction_scope_min").toFloat(), 'f', 2).toUtf8().data();
            root["mapping"]["s1_map_save_max_dis"] = QString::number(map.value("reconstruction_scope_max").toFloat(), 'f', 2).toUtf8().data();
            //[!]标靶0.001 普通0.03
            root["mapping"]["s1_map_save_resolution"] = map.value("MappingModel").toInt()? "0.001" : "0.03";
        }
        if (root["Control_points"]) {
            root["Control_points"]["ControlEnableFlag"] = static_cast<bool>(map.value("ControlEnableFlag").toInt());
            root["Control_points"]["ControlFilePath"] = controlFilePath;
        }
        std::ofstream fout(string_S1FilePath);
        if (!fout.is_open()) {
            std::cerr << "Failed to create output file!\n";
            return false;
        }
        fout << root;
        fout.close();
    }
    else
    {
        std::wstring string_S1FilePath = map.value("FJD_Trion_P1.yaml").toStdWString();
        std::ifstream fin(string_S1FilePath);
        if (!fin.is_open()) {
            std::cerr << "Failed to open file!\n";
            return false;
        }
        YAML::Node root = YAML::Load(fin);
        if (root["Control_points"]) {
            root["Control_points"]["ControlEnableFlag"] = static_cast<bool>(map.value("ControlEnableFlag").toInt());
            root["Control_points"]["ControlFilePath"] = controlFilePath;
        }
        if (root["mapping"]) {
            root["mapping"]["s1_map_save_min_dis"] = QString::number(map.value("reconstruction_scope_min").toFloat(), 'f', 2).toUtf8().data();
            root["mapping"]["s1_map_save_max_dis"] = QString::number(map.value("reconstruction_scope_max").toFloat(), 'f', 2).toUtf8().data();
        }
        std::ofstream fout(string_S1FilePath);
        if (!fout.is_open()) {
            std::cerr << "Failed to create output file!\n";
            return false;
        }
        fout << root;
        fout.close();
    }


    //  if (map.value("product_Type") == "1")
    //  {
          //if (!createS1YamlFiles(map))
          //{
          //	return false;
          //}
    //  }
    //  else
    //  {
          //if (!createP1YamlFiles(map))
          //{
          //	return false;
          //}
    //  }
    return true;
}

void CS::RosMemory::RosMemorySharing::createYamlFiles(QMap<QString, QString>map, std::istream& input)
{
    int slamMode = std::atoi(std::string(map.value("MODE").toLocal8Bit()).c_str());
    float ignoreStartTime = std::atof(std::string(map.value("START").toLocal8Bit()).c_str());
    float ignoreEndTime = std::atof(std::string(map.value("END").toLocal8Bit()).c_str());
    //[!]工程名
    QByteArray Array = map.value("NAME").toUtf8();
    std::string projectName = Array.data();
    removeTrailingNewline(projectName);
    Array.clear();
    //[!]保存路径
    Array = QDir::toNativeSeparators(map.value("SAVE")).toUtf8() + "\\";
    std::string projectSavePath = Array.data();
    removeTrailingNewline(projectSavePath);
    Array.clear();
    //[!]文件打开目录
    Array = QDir::toNativeSeparators(map.value("FILENAME")).toUtf8();
    std::string fjslam_filename = "fjSlamTempFile";
    removeTrailingNewline(fjslam_filename);
    Array.clear();
    //[!]fjslam文件路径
    Array = QDir::toNativeSeparators(map.value("fjslamYAML")).toUtf8();
    std::string fjalamfilepath = Array.data();
    removeTrailingNewline(fjalamfilepath);
    Array.clear();
    //[!]Correctionfilepath
    Array = QDir::toNativeSeparators(map.value("Correction")).toUtf8();
    std::string correctionfilepath = Array.data();
    removeTrailingNewline(correctionfilepath);
    Array.clear();
    //[!]temp Path
    Array = QDir::toNativeSeparators(map.value("TEMPPATH")).toUtf8() + "\\";
    std::string tempPath = Array.data();
    removeTrailingNewline(tempPath);
    Array.clear();
    //[!]gpsEnableFlag
    bool gpsEnableFlagModel = map.value("gpsEnableFlag").toInt();
    //[!]controlFilePath
    std::string controlFilePath = map.value("ControlFilePath").toUtf8().data();
    removeTrailingNewline(controlFilePath);
    Array.clear();

    YAML::Node write_fjslam_params_yaml;
    write_fjslam_params_yaml["project_name"] = projectName;   //工程名称
    write_fjslam_params_yaml["project_savePath"] = projectSavePath; //文件保存路径
    write_fjslam_params_yaml["slam_mode"] = 2; // 建图模式
    write_fjslam_params_yaml["ignore_startTime"] = 0;//[!]开始忽略时间
    write_fjslam_params_yaml["ignore_endTime"] = 0;//[!]结束忽略
    write_fjslam_params_yaml["reconstruction_scope_min"] = map.value("reconstruction_scope_min").toFloat(); //[!]重建范围
    write_fjslam_params_yaml["reconstruction_scope_max"] = map.value("reconstruction_scope_max").toFloat(); //[!]重建范围
    write_fjslam_params_yaml["fjslam_filename"] = fjslam_filename; //文件名不加后缀
    write_fjslam_params_yaml["total_pcdNum"] = map.value("SIZE").toInt();
    write_fjslam_params_yaml["project_tempPath"] = tempPath;
    write_fjslam_params_yaml["fjslamfilepath"] = fjalamfilepath;//[!]fjslam文件路径
    write_fjslam_params_yaml["correctionfilepath"] = correctionfilepath;//[!]correction文件路径
    write_fjslam_params_yaml["gpsEnableFlag"] = gpsEnableFlagModel;
    write_fjslam_params_yaml["ControlEnableFlag"] = static_cast<bool>(map.value("ControlEnableFlag").toInt());				//[!]ControlEnableFlag
    if (map.value("ControlEnableFlag").toInt()) {
        write_fjslam_params_yaml["ControlFilePath"] = controlFilePath;		//[!]ControlFilePath
    }
    QFile file(map.value("YAMLPATH"));
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        std::stringstream yamlStream;
        yamlStream << write_fjslam_params_yaml; 
        std::string yamlData = yamlStream.str();  
        QByteArray byteArray = QByteArray::fromStdString(yamlData);  
        out << QString::fromUtf8(byteArray);
        file.close();
        qDebug() << "mappingConfigFile:" << QString::fromUtf8(byteArray);
    }
    else {
        qDebug() << "Failed to open the file for writing: " << map.value("PATH");
    }

}

void CS::RosMemory::RosMemorySharing::createP1YamlFiles(QMap<QString, QString> map, std::istream& input)
{
    //[!]工程名
    QByteArray Array = map.value("ProjectName").toUtf8();
    std::string projectName = Array.data();
    Array.clear();
    Array = map.value("ProjectSavePath").toUtf8();
    std::string projectSavePath = Array.data();
    Array.clear();
    Array = map.value("ProjectTempPath").toUtf8();
    std::string projectTempPath = Array.data();
    Array.clear();
    Array = map.value("FJDTrionP1ProfilePath").toUtf8();
    std::string FJDTrionP1ProfilePath = Array.data();
    Array.clear();
    Array = map.value("FJDTrionP1FjSlamPath").toUtf8();
    std::string FJDTrionP1FjSlamPath = Array.data();
    Array.clear();

    YAML::Node write_fjslam_params_yaml;
    write_fjslam_params_yaml["project_name"] = projectName;   //工程名称
    write_fjslam_params_yaml["project_savePath"] = projectSavePath; //文件保存路径
    write_fjslam_params_yaml["project_tempPath"] = projectTempPath; //[!]临时文件地址
    write_fjslam_params_yaml["FJDTrionP1ProfilePath"] = FJDTrionP1ProfilePath; //[!]配置文件地址
    write_fjslam_params_yaml["FJDTrionP1FjSlamPath"] = FJDTrionP1FjSlamPath; //[!]fjslam地址

    QFile file(map.value("mapProfilePath"));
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        std::stringstream yamlStream;
        yamlStream << write_fjslam_params_yaml;
        std::string yamlData = yamlStream.str();
        QByteArray byteArray = QByteArray::fromStdString(yamlData);
        out << QString::fromUtf8(byteArray);
        file.close();
        qDebug() << "mapping:" << QString::fromUtf8(byteArray);
    }
    else {
        qDebug() << "Failed to open the file for writing: " << map.value("PATH");
    }

}

void CS::RosMemory::RosMemorySharing::setSharedMemory(std::string share_buffer)
{
    hFileMap_Handle = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_EXECUTE_READWRITE, 0, share_buffer.length() + 1, PROCESS_NAME);
    pBuffer = ::MapViewOfFile(hFileMap_Handle, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    strcpy_s((char*)pBuffer, strlen(share_buffer.c_str()) + 1, share_buffer.c_str());
}

char* CS::RosMemory::RosMemorySharing::getSharedMemory()
{
    hFileMap_Handle = OpenFileMapping(FILE_MAP_ALL_ACCESS, NULL, PROCESS_NAME);
    if (hFileMap_Handle)
    {
        pBuffer = ::MapViewOfFile(hFileMap_Handle, FILE_MAP_ALL_ACCESS, 0, 0, 0);
        return (char *)pBuffer;
    }
    else
    {
        return (char *)"Shared memory error!!!";
    }
}

void  CS::RosMemory::RosMemorySharing::closeSharedMemory()
{
    UnmapViewOfFile(pBuffer);
    CloseHandle(hFileMap_Handle);
}


void CS::RosMemory::RosMemorySharing::deepCopyChar(char* put, char *out)
{
    memcpy(out, put, strlen(put));
}
