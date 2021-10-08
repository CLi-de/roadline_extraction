// ���ݸ����ľ��ȣ�����UTMͶӰ�����
extern "C" _declspec(dllexport)
int tlc_UTM_zone(double lon_in_degree);

// ��XYZ����ת���ɾ�γ������
extern "C" _declspec(dllexport)
bool tlc_XYZ2LL(double X_in_meter, double Y_in_meter, double Z_in_meter,
            double &lon_in_degree, double &lat_in_degree, double &height_in_meter);

// ����γ������ת����XYZ����
extern "C" _declspec(dllexport)
bool tlc_LL2XYZ(double lon_in_degree, double lat_in_degree, double height_in_meter,
            double &X_in_meter, double &Y_in_meter, double &Z_in_meter);

// ����γ������ת��ΪUTM����
extern "C" _declspec(dllexport)
bool tlc_LL2UTM(double lon_in_degree, double lat_in_degree,
            double &easting_in_meter, double &northing_in_meter);

// ��UTM����ת��Ϊ��γ������
extern "C" _declspec(dllexport)
bool tlc_UTM2LL(int zone_num,
            double easting_in_meter, double northing_in_meter,
            double &lon_in_degree, double &lat_in_degree);

// ��XYZ����ת����UTM����
extern "C" _declspec(dllexport)
bool XYZ2UTM(double X_in_meter, double Y_in_meter, double Z_in_meter,
            double &easting_in_meter, double &northing_in_meter, double &height_in_meter);

// ��UTM����ת����XYZ����
extern "C" _declspec(dllexport)
bool UTM2XYZ(int zone_num,
            double easting_in_meter, double northing_in_meter, double height_in_meter,
            double &X_in_meter, double &Y_in_meter, double &Z_in_meter);
