#include <iostream>
#include <vector>

using namespace std;

int main()
{
    vector <int> vct(10);
    int i;

    for(i=0; i < vct.size(); i++){
        vct[i] = 0;
        cout << "vct["<<i<<"]"<< vct[i] << endl;
    }
  

    vct.resize(15);

    for(i=0; i < vct.size(); i++){
        vct[i] = (i+1)*2;
        cout << vct[i] << endl;
    }
    return(0);


}