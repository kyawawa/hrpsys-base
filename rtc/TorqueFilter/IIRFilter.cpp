#include "IIRFilter.h"

IIRFilter::IIRFilter(unsigned int dim, std::vector<double>& fb_coeffs, std::vector<double>& ff_coeffs, const std::string& error_prefix)
{
    std::cerr << "This IIRFilter constructure is obsolated method." << std::endl;
    m_dimension = dim;
    m_error_prefix = error_prefix;

    // init coefficients
    if(fb_coeffs.size() != dim + 1|| ff_coeffs.size() != dim + 1){
        std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter coefficients size error" << std::endl;
        return;
    }
    for(std::vector<double>::iterator it = fb_coeffs.begin(); it != fb_coeffs.end(); it++){
        m_fb_coefficients.push_back(*it);
    }
    for(std::vector<double>::iterator it = ff_coeffs.begin(); it != ff_coeffs.end(); it++){
        m_ff_coefficients.push_back(*it);
    }

    // init frequency
    m_cutoff_freq = 10.0;
    m_hz = 500.0;

    // init previous values
    m_previous_values.assign(dim, 0.0);
    m_initialized = true;
    return;
}

IIRFilter::IIRFilter(const std::string& error_prefix) :
    m_initialized(false), m_hz(500.0), m_cutoff_freq(10.0) {
    m_error_prefix = error_prefix;
}

bool IIRFilter::setParameter(int dim, std::vector<double>& A, std::vector<double>& B) {
    m_dimension = dim;

    // init coefficients
    if((A.size() != dim && A.size() != dim + 1) || B.size() != dim + 1) {
        std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter coefficients size error" << std::endl;
        return false;
    }

    // clear previous coefficients
    m_fb_coefficients.clear();
    m_ff_coefficients.clear();

    if (A.size() == dim) {
        m_fb_coefficients.push_back(1.0);
    }
    for(std::vector<double>::iterator it = A.begin(); it != A.end(); it++){
        if (it == A.begin()) {
            if( *it != 1.0 ) {
                std::cout << "[" <<  m_error_prefix << "]" << "IIRFilter : parameter A[0] is not 1.0 !!!" << std::endl;
            }
            m_fb_coefficients.push_back(*it);
        } else {
            m_fb_coefficients.push_back(- *it);
        }
    }
    for(std::vector<double>::iterator it = B.begin(); it != B.end(); it++){
        m_ff_coefficients.push_back(*it);
    }

    // init previous values
    m_previous_values.assign(dim, 0.0);
    m_initialized = true;
    return true;
}

bool IIRFilter::setSecondButterworthParameter(const double fc_in)
{
    m_cutoff_freq = fc_in;
    double fc = tan(fc_in * M_PI / m_hz) / (2 * M_PI);
    double denom = 1 + (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc;
    std::vector<double> ff_coeffs(3), fb_coeffs(3);
    ff_coeffs[0] = ff_coeffs[2] = (4 * M_PI * M_PI * fc*fc) / denom;
    ff_coeffs[1] = (8 * M_PI * M_PI * fc*fc) / denom;
    fb_coeffs[0] = 1.0;
    fb_coeffs[1] = (8 * M_PI * M_PI * fc*fc - 2) / denom;
    fb_coeffs[2] = (1 - (2 * sqrt(2) * M_PI * fc) + 4 * M_PI * M_PI * fc*fc) / denom;
    return setParameter(2, fb_coeffs, ff_coeffs);
}

void IIRFilter::getParameter(int &dim, std::vector<double>& A, std::vector<double>& B)
{
    dim = m_dimension;
    B.resize(m_ff_coefficients.size());
    std::copy(m_ff_coefficients.begin(), m_ff_coefficients.end(), B.begin());
    A.resize(0);
    for(std::vector<double>::iterator it = m_fb_coefficients.begin();
        it != m_fb_coefficients.end(); it++) {
        if (it == m_fb_coefficients.begin()) {
            A.push_back(*it);
        } else {
            A.push_back(- *it);
        }
    }
}

void IIRFilter::reset(double initial_input)
{
    m_previous_values.assign(m_dimension, initial_input);
}

double IIRFilter::passFilter(double input)
{
    if (! m_initialized) {
        return 0.0;
    }
    double feedback, filtered;
    // calcurate retval
    feedback = m_fb_coefficients[0] * input;
    for (int i = 0; i < m_dimension; i++) {
        feedback += m_fb_coefficients[i + 1] * m_previous_values[i];
    }
    filtered = m_ff_coefficients[0] * feedback;
    for(int i = 0; i < m_dimension; i++) {
        filtered += m_ff_coefficients[i + 1] * m_previous_values[i];
    }
    // update previous values
    m_previous_values.push_front(feedback);
    m_previous_values.pop_back();

    return filtered;
}
