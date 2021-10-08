// 根据给定的经度，计算UTM投影带编号
extern "C" _declspec(dllexport)
int tlc_UTM_zone(double lon_in_degree);

// 将XYZ坐标转换成经纬度坐标
extern "C" _declspec(dllexport)
bool tlc_XYZ2LL(double X_in_meter, double Y_in_meter, double Z_in_meter,
            double &lon_in_degree, double &lat_in_degree, double &height_in_meter);

// 将经纬度坐标转换成XYZ坐标
extern "C" _declspec(dllexport)
bool tlc_LL2XYZ(double lon_in_degree, double lat_in_degree, double height_in_meter,
            double &X_in_meter, double &Y_in_meter, double &Z_in_meter);

// 将经纬度坐标转换为UTM坐标
extern "C" _declspec(dllexport)
bool tlc_LL2UTM(double lon_in_degree, double lat_in_degree,
            double &easting_in_meter, double &northing_in_meter);

// 将UTM坐标转换为经纬度坐标
extern "C" _declspec(dllexport)
bool tlc_UTM2LL(int zone_num,
            double easting_in_meter, double northing_in_meter,
            double &lon_in_degree, double &lat_in_degree);

// 将XYZ坐标转换成UTM坐标
extern "C" _declspec(dllexport)
bool XYZ2UTM(double X_in_meter, double Y_in_meter, double Z_in_meter,
            double &easting_in_meter, double &northing_in_meter, double &height_in_meter);

// 将UTM坐标转换成XYZ坐标
extern "C" _declspec(dllexport)
bool UTM2XYZ(int zone_num,
            double easting_in_meter, double northing_in_meter, double height_in_meter,
            double &X_in_meter, double &Y_in_meter, double &Z_in_meter);
