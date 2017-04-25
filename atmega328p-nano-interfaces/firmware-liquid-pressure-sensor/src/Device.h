#ifndef _Device_H_
#define _Device_H_


template<class S, class D, class I, class C>
class Device {
public:
    virtual void setup(S& setup) = 0;
    virtual void initiateUpdateData() = 0;
    virtual bool isUpdatedDataReady() = 0;
    virtual D& updateData(D& data) = 0;
    virtual I& updateInfo(I& info) = 0;
    virtual bool configure(C& configure) = 0;
};


#endif
