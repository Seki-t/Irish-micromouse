
struct Foot{
	int cost;
	Position pos;
	
	Foot(){}
	Foot(int c, Position p) : cost(c), pos(p) {}
	
	//>にはconstつけないと，stlで使えない

	bool operator > (Foot f)const{
		return cost > f.cost;
	}
	bool operator >= (Foot f)const{
		return cost >= f.cost;
	}
	bool operator < (Foot f)const{
		return cost < f.cost;
	}
	bool operator <= (Foot f)const{
		return cost <= f.cost;
	}
};