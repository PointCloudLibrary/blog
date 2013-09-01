template <typename T>
  class A{
  public:
      void funcInt();
  };
  
  template <> void
  A<PointXYZI>::func()
  {
  }
  
  int main(int argc, char** argv)
  {
    A<PointXYZ> a;
	a.func(); // The linking won't be successful!
	A<PointXYZI> b;
	b.func(); // Fine.
  }