#ifndef PARAMETER_CONFIG_H
#define PARAMETER_CONFIG_H

#include <iostream>
#include <string>
#include <libconfig.h++>

// 参数设置类，辅助作用
class parameterConfig
{
    std::string configure_file_;
    libconfig::Config cfg_;
public:
    parameterConfig()
        :configure_file_("")
    {}

    parameterConfig(const char* file)
    {
        this->setConfigureFile(file);
    }

    // 将value的值设置成参数设置文件中key对应的值
    // 这样写比较灵活，哪里需要设置参数就在哪里使用，不需要写一个复杂的类来承装所有参数
    // 目前不具备 设置一个数组的功能
    template<typename Scalar>
    void get(const char* key, Scalar &value);

    //------------------------------------------------------------
    // 设定参数设置文档，并载入文档
    void setConfigureFile(const char* file)
    {
        configure_file_ = file;
        using namespace std;

        // Read the file. If there is an error, report it and exit.
        try
        {
            cfg_.readFile(configure_file_.c_str());
        }
        catch(const libconfig::FileIOException &fioex)
        {
            cerr << "I/O error while reading file." << endl;
            cerr << fioex.what() << endl;
        }
        catch(const libconfig::ParseException &pex)
        {
            cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                 << " - " << pex.getError() << endl;
        }
    }
};

// 将value的值设置成参数设置文件中key对应的值
// 将模板定义在头文件中。
template<typename Scalar>
void parameterConfig::get(const char *key, Scalar &value)
{
    using namespace std;

    try
    {
        const libconfig::Setting& root = cfg_.getRoot();
        if(root.exists(key))
            root.lookupValue(key, value);
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        cerr << "some settings not found, default value will be used." << endl;
        cerr << nfex.what() << endl;
    }
}

#endif // PARAMETER_CONFIG_H
