

namespace sick_lidar
{
    class DebugUtil
    {
    public:
        static void printArray(const char* arr, int size)
        {
            QByteArray array(arr,size);
            qDebug() << QString(array.toHex());
        }
    };
}
