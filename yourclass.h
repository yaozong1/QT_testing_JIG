class YourClass : public QWidget, public Dialog // 或者其他适当的基类
{
    Q_OBJECT

public:
    YourClass(QWidget *parent = nullptr);
    ~YourClass();
    void newTest(void);

private:
    // 声明私有函数
    void performJLinkProgramming();

    // 其他成员变量和内容
};
