#include <iostream>
#include <string>
#include <tuple>

template <typename... Args>
class MyClass
{
public:
    MyClass() {}

    void setValues(Args... args)
    {
        int i = 0;
        ((values[i++] = args), ...); // assign each argument to the 'values' array
    }

    void printValues()
    {
        for (auto &v : values)
        {
            std::cout << v;
        }
        // for (int i = 0; i < std::tuple_size<Args...>::value; ++i)
        // {
        //     std::cout<<std::get<i>(values);
        // }
        // std::cout << std::endl;
    }

private:
    std::tuple<Args...> values;
};

int main()
{
    MyClass<double, double, std::string> myObj1;

    myObj1.setValues(1, 3.14, "hello");
    myObj1.printValues(); // output: 1 3.14 hello

    MyClass<double, double> myObj2;
    myObj2.setValues(42, 2.718);
    myObj2.printValues(); // output: 42 2.718

    return 0;
}
