#include "ColorBasedParticleFilter.h"

ColorBasedParticleFilter::ColorBasedParticleFilter(int Width, int Height) {
	for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {
		particles.push_back(Particle(Width, Height));
	}
}

ColorBasedParticleFilter::ColorBasedParticleFilter(int X, int Y, int Width, int Height) {
	for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {
		particles.push_back(Particle(X, Y, Width, Height));
	}
}

void ColorBasedParticleFilter::on_newFrame(Mat* m) {
	double weights_sum = 0;
	double max_raw_weight = 0;

	for (int i = 0; i < particles.size(); i++) {
		Particle* p = &particles[i];
		double weight = calc_weight_for_particle(p, *m);
		p->weight = weight;
		weights_sum += weight;

		//Get the maximum assigned weight
		if (weight > max_raw_weight)
			max_raw_weight = weight;
	}

	//Normalize the weights
	for (int i = 0; i < particles.size(); i++) {
		Particle* p = &particles[i];
		p->weight = p->weight / weights_sum;
	}
	vector<double> weighted_distribution = get_weighted_distribution(particles);

	//Re-sample the particles
	vector<Particle> new_particles_list;
	for (int i = 0; i < particles.size(); i++){
		Particle* resampled_particle = get_new_particle(weighted_distribution);
		new_particles_list.push_back(*resampled_particle);
	}
	particles.clear();
	particles = new_particles_list;

	calculate_particles_xy_mean();

	move_particle();
	
	//Resize the tracking window size
	if (step < 6)
		step++;
	else{
		step = 0;
		if (feature_point_count == 0) {
			Mat t = *m;
			Mat sub_mat = t(Rect(mean_x, mean_y, tracking_window_width, tracking_window_height));
			feature_point_count = calc_first_class_points(sub_mat);
		}
		else {
			Mat t = *m;
			Mat sub_mat = t(Rect(mean_x,mean_y,(mean_x+tracking_window_width >= m->cols)?m->cols-mean_x:tracking_window_width,(mean_y+tracking_window_height>=m->rows)?m->rows-mean_y:tracking_window_height));
			int p = calc_first_class_points(sub_mat);

			if (p > feature_point_count * 1.1) {
				feature_point_count = p;
				tracking_window_width += 10;
				tracking_window_height += 10;
				if (tracking_window_width > initial_tracking_window_width * 1.5)
					tracking_window_width = initial_tracking_window_width * 1.5;
				if (tracking_window_height > initial_tracking_window_height * 1.5)
					tracking_window_height = initial_tracking_window_height * 1.5;
			}
			else if (p < feature_point_count * 0.9) {
				feature_point_count = p;
				tracking_window_width -= 10;
				tracking_window_width -= 10;
				if (tracking_window_width < initial_tracking_window_width * 0.5)
					tracking_window_width = initial_tracking_window_width * 0.5;
				if (tracking_window_height < initial_tracking_window_height * 0.5)
					tracking_window_height = initial_tracking_window_height * 0.5;
			}

			double w = tracking_window_width;
			double h = tracking_window_height;
			double wd = initial_tracking_window_width;
			double hd = initial_tracking_window_height;

			h = w * (hd / wd);
			tracking_window_height = h;
		}
	}

	//Draw the tracking rectangle
	Scalar* line_color;
	if (max_raw_weight > 0.5)
		line_color = new Scalar(0, 255, 0);
	else if (max_raw_weight > 0.2)
		line_color = new Scalar(255, 0, 0);
	else
		line_color = new Scalar(0, 0, 255);
	rectangle(*m, Rect(mean_x,mean_y,tracking_window_width,tracking_window_height), *line_color, 3);
	
	//Draw the particles
	for (int i = 0; i < particles.size(); i++) {
		Particle* p = &particles[i];
		line(*m, Point(p->x, p->y), Point(p->x, p->y), Scalar(0, 255, 0), 7);
	} 
}

void ColorBasedParticleFilter::set_from_initial_frame(Mat m, int x1, int y1, int x2, int y2) {
	vector<Mat> bgr_channels(3);

	//Important!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	Rect rect(Point(x1, y1), Point(x2, y2));
	Mat submat = m(rect);
	split(submat, bgr_channels);

	initial_frame_hist_b = calculate_histogram(bgr_channels[0]);
	initial_frame_hist_g = calculate_histogram(bgr_channels[1]);
	initial_frame_hist_r = calculate_histogram(bgr_channels[2]); 
}  

Mat ColorBasedParticleFilter::calculate_histogram(Mat m) {
	Mat hist;
	int histSize = 128;
	float range[] = { 0, 256 };
	const float* histRange = { range };

	calcHist(&m, 1, 0, Mat(), hist, 1, &histSize, &histRange, true, false);

	return hist;
}

void ColorBasedParticleFilter::calculate_particles_xy_mean() {
	mean_x_in_previous_frame = mean_x;
	mean_y_in_previous_frame = mean_y;

	mean_x = 0;
	mean_y = 0;

	for (int i = 0; i < particles.size(); i++) {
		Particle* p = &particles[i];
		mean_x += p->x;
		mean_y += p->y;
	}
	mean_x = mean_x / particles.size();
	mean_y = mean_y / particles.size();
}

double ColorBasedParticleFilter::calc_weight_for_particle(Particle* p, Mat m) {
	double weight;
	int x = (int)(p->x + 0.5);
	int y = (int)(p->y + 0.5);

	int x_end = x + tracking_window_width;
	int y_end = y + tracking_window_height;

	if (x_end >= image_width) x_end = image_width - 1;
	if (y_end >= image_height) y_end = image_height - 1;
	if (x < 0) { x = 0; }
	if (y < 0) { y = 0; }

	if (x_end <= x || y_end <= y) {
		return 0;
	}

	Rect rect(Point(x, y), Point(x_end, y_end));
	Mat submat = m(rect);
	vector<Mat> bgr_channels(3);
	split(submat, bgr_channels);

	Mat hist_b = calculate_histogram(bgr_channels[0]);
	Mat hist_g = calculate_histogram(bgr_channels[1]);
	Mat hist_r = calculate_histogram(bgr_channels[2]);

	double correlation_b = compareHist(hist_b, initial_frame_hist_b, 0);
	double correlation_g = compareHist(hist_g, initial_frame_hist_g, 0);
	double correlation_r = compareHist(hist_r, initial_frame_hist_r, 0);

	double par = 0.33 * (correlation_b + correlation_g + correlation_r);
	weight = exp(-16 * (1 - par));

	return weight;
}

vector<double> ColorBasedParticleFilter::get_weighted_distribution(vector<Particle> p) {
	vector<double> weighted_distribution;

	double accumulation = 0;
	for (int i = 0; i < p.size(); i++) {
		Particle* t = &p[i];
		accumulation += t->weight;

		weighted_distribution.push_back(accumulation);
	}

	return weighted_distribution;
}

Particle* ColorBasedParticleFilter::get_new_particle(vector<double> weighted_distribution) {
	Particle* new_particle = new Particle();
	double number = ((double)rand() / (double)RAND_MAX);

	for (int i = 0; i < particles.size(); i++) {
		if (i == 0) {
			if (number <= weighted_distribution[0]) {
				new_particle->x = particles[i].x;
				new_particle->y = particles[i].y;
				break;
			}
		}
		else if (i == particles.size() - 1) {
			new_particle->x = particles[i].x;
			new_particle->y = particles[i].y;
			break;
		}
		else {
			if (number <= weighted_distribution[i] && number > weighted_distribution[i - 1]) {
				new_particle->x = particles[i].x;
				new_particle->y = particles[i].y;
				break;
			}
		}
	}

	return new_particle;
}

void ColorBasedParticleFilter::move_particle() {

	for (int i = 0; i<particles.size(); i++) {
		Particle* particle = &particles[i];
		double dx = 50 * ((double)rand() / (double)RAND_MAX) - 25.0;
		double dy = 50 * ((double)rand() / (double)RAND_MAX) - 25.0;

		particle->x = particle->x + dx;
		particle->y = particle->y + dy;

		if (particle->x <= 0) particle->x = 0;
		if (particle->x >= image_width) particle->x = image_width - 1;
		if (particle->y <= 0) particle->y = 0;
		if (particle->y >= image_height) particle->y = image_height - 1;
	}
}

void ColorBasedParticleFilter::set_image_size(int w, int h) {
	image_width = w;
	image_height = h;
}

void ColorBasedParticleFilter::set_tracking_window(int w, int h) {
	tracking_window_width = w;
	tracking_window_height = h;
}


