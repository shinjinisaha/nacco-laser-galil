#include<iostream>
#include <fstream> 
#include <string>
using namespace std;

int main()
{
ifstream fin("param.txt");
string param_name;
int param_value;
while ( fin >> param_name >> param_value )
{
  if (!param_name.compare("angles_defined_x")){
  cout <<" param_value is " << param_value << "  and name is " << param_name << "\n" ;}
}
return 0;
}
