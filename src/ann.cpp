#include <iostream>
#include <random>
#include <vector>
#include <fstream>

#include "ann.h"

std::random_device dev;
std::mt19937 rng(dev());

double get_random_double(double min, double max) {
	std::uniform_real_distribution<double> unid(min, max);
	return unid(rng);
}

int get_random_int(int min, int max) {
	std::uniform_int_distribution<int> unii(min, max);
	return unii(rng);
}

DenseLayer::DenseLayer(unsigned int in, unsigned int out, bool use_bias) {
    inputs = in;
    outputs = out;
    bias = use_bias;

    w.resize(inputs);
    for (unsigned int j = 0; j < outputs; ++j) {
        for (unsigned int i = 0; i < inputs; ++i) {
            w.at(j).push_back(get_random_double(-1, 1));
        }
        b.push_back(get_random_double(-1, 1));
    }
}

DenseLayer::DenseLayer(const DenseLayer& other) {
    inputs = other.inputs;
    outputs = other.outputs;
    w = other.w;
    b = other.b;
    bias = other.bias;
}

DenseLayer& DenseLayer::operator=(const DenseLayer& other) {
    if (this != &other) {
        *this = DenseLayer(other);
        return *this;
    }
    return *this;
}

void DenseLayer::print() {
    for (unsigned int j = 0; j < outputs; j++) {
        for (unsigned int i = 0; i < inputs; i++) {
            if (i > 0)
                std::cout << ",";
            std::cout << w.at(j).at(i);
        }
        if (bias)
            std::cout << " + " << b.at(j);
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

std::vector<double> DenseLayer::call(std::vector<double> x) {
    std::vector<double> result(outputs, 0);

    for (unsigned int j = 0; j < outputs; j++) {
        if (bias)
            result.at(j) += b.at(j);
        for (unsigned int i = 0; i < inputs; i++) {
            result.at(j) += w.at(j).at(i) * x.at(i);
        }
    }
    return result;
}

void DenseLayer::mutate() {
    double step_size = 0;

    for (unsigned int j = 0; j < outputs; j++) {
        step_size = get_random_double(0, 0.1);
        b.at(j) += get_random_double(-step_size, step_size);
        for (unsigned int i = 0; i < inputs; i++) {
            step_size = get_random_double(0, 0.1);
            w.at(j).at(i) += get_random_double(-step_size, step_size);
        }
    }
}

std::ostream& operator<<(std::ostream& os, const DenseLayer& l)
{
    os << "{\"weights\":[";
    for (unsigned int j = 0; j < l.outputs; ++j) {
        if (j > 0)
            os << ",";
        os << "[";
        for (unsigned int i = 0; i < l.inputs; ++i) {
            if (i > 0)
                os << ",";
            os << l.w.at(j).at(i);
        }
        os << "]";
    }
    os << "],\"bias\":[";
    for (unsigned int j = 0; j < l.outputs; ++j) {
        if (j > 0)
            os << ",";
        os << l.b.at(j);
    }
    os << "]}";
    return os;
}

ANN::ANN(int in, int out) {
    layer.push_back(DenseLayer(in, 2, true));
    layer.push_back(DenseLayer(2, out, true));
}

ANN::ANN(std::vector<DenseLayer>& initialLayers) : layer(initialLayers) {}

// Copy constructor.
ANN::ANN(const ANN& other) {
    layer = other.layer;
}

// Assignment operator.
ANN& ANN::operator=(const ANN& other) {
    if (this != &other) {
        *this = ANN(other);
        return *this;
    }
    return *this;
}

void ANN::print() {
    for (int i = 0; i < layer.size(); i++) {
        std::cout << "Layer " << i << ":" << std::endl;
        layer.at(i).print();
    }
}

void ANN::saveToFile() {
    std::ofstream file;
    file.open("model.json");
    file << "[";
    for (int i = 0; i < layer.size(); i++) {
        if (i > 0)
            file << ",";
        file << layer.at(i);
    }
    file << "]";
    file.close();
}

std::vector<double> ANN::call(std::vector<double> x) {
    for (int i = 0; i < layer.size(); i++) {
        x = layer.at(i).call(x);
    }
    for (int j = 0; j < x.size(); j++) {
        // Different version of tanh between -5 and 5 with a moderate curve.
        x.at(j) = -5 + 10 / (1 + exp(-1 * x.at(j)));
    }
    return x;
}

void ANN::mutate() {
    for (unsigned int i = 0; i < layer.size(); i++) {
        layer.at(i).mutate();
    }
}

ANNGenerator::ANNGenerator(unsigned int hidden_layers, unsigned int hidden, unsigned int in, unsigned int out, bool use_bias) :
    hidden_layers(hidden_layers),
    hidden_size(hidden),
    input_size(in),
    output_size(out),
    bias(use_bias) {}

void ANNGenerator::fill_random_population(std::vector<ANN>& population, unsigned int size) {
    for (unsigned int p = 0; p < size; p++) {
        std::vector<DenseLayer> layer;
        if (0 == hidden_layers) {
            layer.push_back(DenseLayer(input_size, output_size, bias));
        }
        else {
            layer.push_back(DenseLayer(input_size, hidden_size, bias));
            for (unsigned int l = 1; l < hidden_layers; l++) {
                layer.push_back(DenseLayer(input_size, hidden_size, bias));
            }
            layer.push_back(DenseLayer(hidden_size, output_size, bias));
        }
        population.push_back(ANN(layer));
    }
}

ANN ANNGenerator::crossover(const std::vector<ANN>& population, unsigned int num_parents, unsigned int max_parent_index) {
    std::vector<int> parents;
    unsigned int max_pop = std::min((static_cast<unsigned int>(population.size()) - 1), max_parent_index);
    
    // Choose parents.
    for (unsigned int p = 0; p < num_parents; p++) {
        parents.push_back(get_random_int(0, max_pop));
    }

    std::vector<DenseLayer> new_layer;
    unsigned int which_parent;

    // Choose parent for each layer.
    for (unsigned int l = 0; l <= hidden_layers; l++) {
        which_parent = get_random_int(0, num_parents-1);
        DenseLayer layer = population.at(parents.at(which_parent)).layer.at(l);
        new_layer.push_back(layer);
    }

    return ANN(new_layer);
}