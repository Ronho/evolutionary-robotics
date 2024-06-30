#include <iostream>
#include <vector>

int get_random_int(int min, int max);

double get_random_double(double min, double max);

class DenseLayer {
public:
	unsigned int inputs, outputs;
	bool bias;
	std::vector<std::vector<double>> w;
	std::vector<double> b;

public:
	DenseLayer(unsigned int in, unsigned int out, bool use_bias);

	DenseLayer(const DenseLayer& other);

	DenseLayer& operator=(const DenseLayer& other);

	void print();

	std::vector<double> call(std::vector<double> x);

	void mutate();

	friend std::ostream& operator<<(std::ostream& os, const DenseLayer& l);
};

class ANN {
public:
	std::vector<DenseLayer> layer;

public:
	ANN(int in, int out);

	ANN(std::vector<DenseLayer>& initialLayers);

	ANN(const ANN& other);

	ANN& operator=(const ANN& other);

	void print();

	void saveToFile();

	std::vector<double> call(std::vector<double> x);

	void mutate();
};

class ANNGenerator {
private:
	unsigned int hidden_layers, hidden_size, input_size, output_size;
	bool bias;

public:
	ANNGenerator(unsigned int hidden_layers, unsigned int hidden, unsigned int in, unsigned int out, bool use_bias);

	void fill_random_population(std::vector<ANN>& population, unsigned int size);

	ANN crossover(const std::vector<ANN>& population, unsigned int num_parents, unsigned int max_parent_index);
};