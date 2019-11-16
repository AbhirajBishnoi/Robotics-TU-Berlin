#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            //return generateBridge();
            return generateGaussian();
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector
        YourSampler::generateGaussian() {
          while (true) {
            // generate first sample
            ::rl::math::Vector rand(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
            }
            ::rl::math::Vector sample1 = this->model->generatePositionUniform(rand);

            // generate second sample (gaussian around the first one)
            ::rl::math::Vector sigma(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
              sigma(i) = 7.5;
            }
            ::rl::math::Vector sample2 = this->model->generatePositionGaussian(rand, sample1, sigma);

            // test if first is colliding
            this->model->setPosition(sample1);
            this->model->updateFrames();

            if(this->model->isColliding()){
              // test if second is colliding
              this->model->setPosition(sample2);
              this->model->updateFrames();
              if(!this->model->isColliding()) {
                // sample2 is free and sample1 colliding
                return sample2;
              }
            } else{
              // test if second is colliding
              this->model->setPosition(sample2);
              this->model->updateFrames();
              if(this->model->isColliding()) {
                // sample1 is free and sample2 colliding
                return sample1;
              }
            }
            // repeat this till valid sample
          }
        }

        ::rl::math::Vector
        YourSampler::generateBridge(){
          while(true) {
            // generate first sample
            ::rl::math::Vector rand(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
            }
            ::rl::math::Vector sample1 = this->model->generatePositionUniform(rand);

            // test if first is colliding
            this->model->setPosition(sample1);
            this->model->updateFrames();
            if (!this->model->isColliding()) {
              continue;
            }

            // generate second sample (gaussian around the first one)
            ::rl::math::Vector sigma(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
              sigma(i) = 50.0;
            }
            ::rl::math::Vector sample2 = this->model->generatePositionGaussian(rand, sample1, sigma);

            // test if second is colliding
            this->model->setPosition(sample2);
            this->model->updateFrames();
            if (!this->model->isColliding()) {
              continue;
            }

            ::rl::math::Vector actual_point(this->model->getDof());

            // interpolate for point in the middle
            this->model->interpolate(sample1, sample2, 0.5f, actual_point);

            // and check if that this is not colliding
            this->model->setPosition(actual_point);
            this->model->updateFrames();
            if(!this->model->isColliding()){
              return actual_point;  // the point is a free point with two colliding points around
            }

          }
        }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
        }
    }
}

