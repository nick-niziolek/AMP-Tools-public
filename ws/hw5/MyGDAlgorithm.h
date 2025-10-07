#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
	private:
		const amp::Problem2D problem;
		
		double d_star, zetta, Q_star, eta;

    public:
		MyPotentialFunction(const amp::Problem2D& prob,const double d_star,const double zetta,const double Q_star,const double eta) : problem(prob), d_star(d_star), zetta(zetta), Q_star(Q_star), eta(eta) {}
		
		virtual double operator()(const Eigen::Vector2d& q) const override;
		
		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override;

		// Get nearest distance and vector to obstacle
		std::vector<std::pair<double, Eigen::Vector2d>> getNearestObstacleInfo(const Eigen::Vector2d& q) const;

		// Get nearest vertex of all obstacles
		Eigen::Vector2d getNearestVertex(const Eigen::Vector2d& q) const;
};