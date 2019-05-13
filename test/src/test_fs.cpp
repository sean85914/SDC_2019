#include <boost/filesystem.hpp>
#include <iostream>

int main()
{
  boost::filesystem::path path("/home/sean/Downloads/SDC/test/haa");
  if(!boost::filesystem::exists(path))
    std::cout << "No such folder\n";
  boost::filesystem::create_directory(path);
  return 0;
}
