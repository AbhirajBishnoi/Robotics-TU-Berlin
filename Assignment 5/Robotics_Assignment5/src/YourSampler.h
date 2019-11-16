#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();

            virtual void seed(const ::std::mt19937::result_type& value);

            ::rl::math::Vector generateGaussian();

            ::rl::math::Vector generateBridge();

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;

            ::std::mt19937 randEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
