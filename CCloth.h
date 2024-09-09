#pragma once

namespace MPMSDF 
{
	class Vector2f;
}


namespace PBD
{

	void DistanceConstraint(MPMSDF::Vector2f& p0, MPMSDF::Vector2f& p1, float w0, float w1, float d, float k=1,int N=10);

	class CCloth
	{
	public:
		CCloth();
		virtual ~CCloth();

		void init();
		void tick();

		std::vector<MPMSDF::Vector2f> mPos;
		std::vector<MPMSDF::Vector2f> mBall;

	private:		
		std::vector<MPMSDF::Vector2f> pPos;
		std::vector<MPMSDF::Vector2f> mVel;

		std::vector<MPMSDF::Vector2f> pBall;
		std::vector<MPMSDF::Vector2f> mBallVel;

		std::vector<float> mInvMass;
		std::vector<float> mBallInvMass;

		std::vector<bool> mCollisionFlag;

		int N;
	};
}



