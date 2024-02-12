#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>
#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dexi_msgs/srv/set_led_effect.hpp"
#include "led_msgs/srv/set_le_ds.hpp"
#include "led_msgs/msg/led_state.hpp"
#include "led_msgs/msg/led_state_array.hpp"

// #include "sensor_msgs/msg/battery_state.h"
// #include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> VALID_EFFECTS = {
	"fill",
	"blink",
	"blink_fast",
	"fade",
	"wipe",
	"flash",
	"rainbow_fill",
	"rainbow"
};

class Effect {
	public:
		Effect(std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> effect, rclcpp::Time curr_time)
			:_effect(effect) {this->combine(std::make_shared<Effect>(effect), curr_time);}

		Effect(std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> effect)
			:_effect(effect), _infinite(true){}

		bool same(const std::shared_ptr<Effect> other) {
			return this->_effect->effect == other->_effect->effect &&
				   this->_effect->r == other->_effect->r &&
				   this->_effect->g == other->_effect->g &&
				   this->_effect->b == other->_effect->b;
		}

		void combine(const std::shared_ptr<Effect> other, rclcpp::Time curr_time) {
			double duration = other->_effect->duration;
			this->_effect = other->_effect;
			if(duration < 1e-3) {
				this->_infinite = true;
			} else {
				this->_end_time = curr_time + std::chrono::duration<double>(duration);
			}
		}

		std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> get_effect() {
			return this->_effect;
		}

		bool finished(rclcpp::Time curr_time) {
			return !this->_infinite && curr_time > this->_end_time;
		}

		std::string to_string() {
			return this->_effect->effect 
					+ " (r:" + std::to_string(this->_effect->r)
					+ ", g:" + std::to_string(this->_effect->g) 
					+ ", b:" + std::to_string(this->_effect->b) + ")"
					+ "[priority:" + std::to_string(this->_effect->priority) + "]";
		}

		private:
			std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> _effect;
			rclcpp::Time _end_time;
			bool _infinite = false;
};

class DexiLEDController : public rclcpp::Node
{
    public:
        DexiLEDController();
		~DexiLEDController();
        void callSetLeds();
        void rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b);
        void fill(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
		void set_leds_index(uint8_t i, uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
        void proceed();
        bool setEffect(std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> req, std::shared_ptr<dexi_msgs::srv::SetLedEffect::Response> res);
        void handleState(const led_msgs::msg::LEDStateArray::SharedPtr msg);
		bool startEffect(std::shared_ptr<Effect> effect);

    private:
		std::pair<std::shared_ptr<Effect>, uint8_t> get_effect_queue();
		void set_effect_queue(uint8_t p_idx, std::shared_ptr<Effect> effect);
		void reset_queue();

		std::shared_ptr<Effect> base_effect;
		std::vector<std::shared_ptr<Effect>> pq;
		std::mutex q_mutex;
		
		std::shared_ptr<Effect> curr_effect;
        std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> current_effect;
		std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> default_base_effect;
        int led_count;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Time start_time;

        double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
		unsigned int default_brightness;
        double low_battery_threshold;
        bool blink_state;
		int flash_number;

		bool swap_red_blue;

		uint32_t num_priority_levels;

        std::shared_ptr<led_msgs::srv::SetLEDs::Request> set_leds;
        std::shared_ptr<led_msgs::msg::LEDStateArray> state, start_state;

        rclcpp::Client<led_msgs::srv::SetLEDs>::SharedPtr set_leds_srv;
        rclcpp::Subscription<led_msgs::msg::LEDStateArray>::SharedPtr state_sub;
        rclcpp::Service<dexi_msgs::srv::SetLedEffect>::SharedPtr set_effect;

		// rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
        // std::shared_ptr<mavros_msgs::msg::State> mavros_state;
        int counter;

		rclcpp::CallbackGroup::SharedPtr callback_group_services_;

		void restartTimer(double seconds);
};

DexiLEDController::DexiLEDController() : 
	Node("led", 
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
	// this->get_node_options().allow_undeclared_parameters(true);
	// this->get_node_options().automatically_declare_parameters_from_overrides(true);
	// double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
	this->get_parameter_or("blink_rate",this->blink_rate, 2.0);
	this->get_parameter_or("blink_fast_rate",this->blink_fast_rate, blink_rate * 4.0);
	this->get_parameter_or("fade_period",this->fade_period, 0.5);
	this->get_parameter_or("wipe_period",this->wipe_period, 0.5);
	this->get_parameter_or("flash_delay",this->flash_delay, 0.1);
	this->get_parameter_or("flash_number",this->flash_number, 1);
	this->get_parameter_or("rainbow_period",this->rainbow_period, 5.0);
	this->get_parameter_or("swap_red_blue", this->swap_red_blue, true);
	this->get_parameter_or("brightness", this->default_brightness, 120U);
	this->get_parameter_or("notify/low_battery/threshold", this->low_battery_threshold, 3.7);
	this->get_parameter_or("num_priority_levels", this->num_priority_levels, 10U);

	// Initialise set leds
	this->state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->start_state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->set_leds = std::make_shared<led_msgs::srv::SetLEDs::Request>();
	this->current_effect = std::make_shared<dexi_msgs::srv::SetLedEffect::Request>();

	// Resize Queue with all initialised to nullptr
	this->pq.resize(this->num_priority_levels, nullptr);

	// New values
	this->default_base_effect = std::make_shared<dexi_msgs::srv::SetLedEffect::Request>();
	this->default_base_effect->effect = "rainbow";
	this->default_base_effect->brightness = this->default_brightness;
	this->base_effect = std::make_shared<Effect>(this->default_base_effect);

	this->callback_group_services_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    // First need to wait for service
    this->set_leds_srv = this->create_client<led_msgs::srv::SetLEDs>("set_leds", rmw_qos_profile_services_default, this->callback_group_services_);
    this->set_leds_srv->wait_for_service(10s);

    this->state_sub = this->create_subscription<led_msgs::msg::LEDStateArray>(
        "led_state", 10,
        std::bind(&DexiLEDController::handleState, this, std::placeholders::_1)
    );

    this->set_effect = this->create_service<dexi_msgs::srv::SetLedEffect>(
        "set_effect",
        std::bind(&DexiLEDController::setEffect, this, std::placeholders::_1, std::placeholders::_2)
    );

	this->restartTimer(0.5);
	RCLCPP_INFO(this->get_logger(), "LED Effects Controller Initialised");
}

DexiLEDController::~DexiLEDController() {
	// Destructor simply turns off lights.
	this->fill(0, 0, 0, 0);
}

void DexiLEDController::set_effect_queue(uint8_t p_idx, std::shared_ptr<Effect> effect) {
	std::lock_guard<std::mutex> guard(this->q_mutex);
	this->pq[p_idx] = effect;
}

std::pair<std::shared_ptr<Effect>, uint8_t> DexiLEDController::get_effect_queue() {
	std::lock_guard<std::mutex> guard(this->q_mutex);
	uint8_t p_idx = this->num_priority_levels - 1;
	std::shared_ptr<Effect> effect;
	for(auto it = this->pq.rbegin(); it != this->pq.rend(); ++it ) {
		if(*it){effect = *it; break;}
		p_idx--;
	}
	return std::make_pair(effect, p_idx);
}

void DexiLEDController::reset_queue(){
	std::lock_guard<std::mutex> guard(this->q_mutex);
	std::fill(this->pq.begin(), this->pq.end(), nullptr); // Probably should lock this...
}

void DexiLEDController::restartTimer(double seconds) 
{
	if (this->timer) {
		this->timer->cancel();
	}
	this->timer = this->create_wall_timer(
		std::chrono::duration<double>(seconds), 
		std::bind(&DexiLEDController::proceed, this)
	);
    RCLCPP_DEBUG(this->get_logger(), "Reset Timer to %f", seconds);
}

void DexiLEDController::callSetLeds()
{
	auto result_future = this->set_leds_srv->async_send_request(this->set_leds);
	if (result_future.wait_for(std::chrono::duration<double>(10.0)) != std::future_status::ready)
	{
		RCLCPP_WARN(this->get_logger(), "LED Call request Failed");
	}
}

void DexiLEDController::set_leds_index(uint8_t i, uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
	this->set_leds->leds[i].index = index;
	if(this->swap_red_blue){
		this->set_leds->leds[i].r = b;
		this->set_leds->leds[i].b = r;
	} else {
		this->set_leds->leds[i].r = r;
		this->set_leds->leds[i].b = b;
	}
	this->set_leds->leds[i].g = g;
	
	this->set_leds->leds[i].brightness = brightness;
}

void DexiLEDController::rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b)
{
	if (n < 255 / 3) {
		r = n * 3;
		g = 255 - n * 3;
		b = 0;
	} else if (n < 255 / 3 * 2) {
		n -= 255 / 3;
		r = 255 - n * 3;
		g = 0;
		b = n * 3;
	} else {
		n -= 255 / 3 * 2;
		r = 0;
		g = n * 3;
		b = 255 - n * 3;
	}
}

void DexiLEDController::fill(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
	this->set_leds->leds.resize(this->led_count);
	for (int i = 0; i < led_count; i++) {
		this->set_leds_index(
			i, i, r, g, b, brightness
		);
	}
	this->callSetLeds();
}

void DexiLEDController::handleState(const led_msgs::msg::LEDStateArray::SharedPtr msg)
{
    this->state = msg;
    this->led_count = this->state->leds.size();
    // RCLCPP_INFO(this->get_logger(), "Handling received led state with %d leds", this->led_count);
}

void DexiLEDController::proceed()
{

	rclcpp::Time curr_time = this->now();

	// Check for current most prioritised effect
	auto ret = this->get_effect_queue();
	std::shared_ptr<Effect> effect = ret.first;
	uint8_t p_idx = ret.second;
	

	// Check if effect is found
	if(effect) {
		// Check if effect has expired, remove the effect from the queue
		if(effect->finished(curr_time)) {
			// this->pq[p_idx] = nullptr;
			this->set_effect_queue(p_idx, nullptr);
		}
	} else {
		// If no effect found, set effect to base effect
		effect = this->base_effect;
	}

	// Check if effect is different from current effect
	if(!this->curr_effect || !this->curr_effect->same(effect)) {
		// Parse and set state for new effect
		RCLCPP_INFO(this->get_logger(), ("Effect change detected, changing effect to " + effect->to_string()).c_str());
		this->startEffect(effect);
		this->curr_effect = effect;
	}

	// Execute effect
	this->current_effect = this->curr_effect->get_effect();
	this->counter++;
	uint8_t r, g, b;
	this->set_leds->leds.clear();
	this->set_leds->leds.resize(this->led_count);

	if (this->current_effect->effect == "blink" || this->current_effect->effect == "blink_fast") {
		RCLCPP_INFO(this->get_logger(), "proceed: blinking");
		this->blink_state = !this->blink_state;
		// toggle all leds
		if (this->blink_state) {
			this->fill(this->current_effect->r, this->current_effect->g, this->current_effect->b, this->current_effect->brightness);
		} else {
			this->fill(0, 0, 0, 0);
		}

	} 
	else if (this->current_effect->effect == "fade") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: fade");
	// 	// fade all leds from starting state
		double time_elapsed = (double) (this->now() - this->start_time).nanoseconds();
		double passed = std::min( time_elapsed / this->fade_period, 1.0);
		double one_minus_passed = 1 - passed;
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds_index(
				i, i,
				one_minus_passed * this->start_state->leds[i].r + passed * this->current_effect->r,
				one_minus_passed * this->start_state->leds[i].g + passed * this->current_effect->g,
				one_minus_passed * this->start_state->leds[i].b + passed * this->current_effect->b,
				one_minus_passed * this->start_state->leds[i].brightness + passed * this->current_effect->brightness
			);
		}
		this->callSetLeds();
		if (passed >= 1.0) {
			// fade finished
			this->timer->cancel();
		}
	} 
	else if (this->current_effect->effect == "wipe") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: wipe");
		this->set_leds->leds.resize(1);
		this->set_leds_index(
			0,
			this->counter - 1,
			this->current_effect->r,
			this->current_effect->g,
			this->current_effect->b,
			this->current_effect->brightness
		);
		this->callSetLeds();
		if (this->counter == this->led_count) {
			// wipe finished
			this->timer->cancel();
		}

	} 
	else if (this->current_effect->effect == "rainbow_fill") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: rainbow_fill");
		this->rainbow(this->counter % 255, r, g, b);
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds_index(i,i,r,g,b, this->current_effect->brightness);
		}
		this->callSetLeds();
	}
	else if (this->current_effect->effect == "rainbow") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: rainbow");
		for (int i = 0; i < this->led_count; i++) {
			int pos = (int)round(this->counter + (255.0 * i / this->led_count)) % 255;
			this->rainbow(pos % 255, r, g, b);
			this->set_leds_index(i,i,r,g,b,this->current_effect->brightness);
		}
		this->callSetLeds();
	}
}

bool DexiLEDController::setEffect(std::shared_ptr<dexi_msgs::srv::SetLedEffect::Request> req, std::shared_ptr<dexi_msgs::srv::SetLedEffect::Response> res)
{
    res->success = true;


	// Set Defaults
	if (req->effect == "") {req->effect = "fill";}
	if (!req->brightness) {req->brightness = this->default_brightness;}

	RCLCPP_INFO(
		this->get_logger(), 
		"Received led set effect: %s (r: %i, g: %i, b: %i) brightness: %i, duration: %f, priority %i", 
		req->effect.c_str(), req->r, req->g, req->b, req->brightness, req->duration, req->priority
	);

	if(!req->priority) {
		req->priority = 0;
	}

	if(req->priority > this->num_priority_levels) {
		res->message = "Requested priority is greater than allowed priority";
		res->success = false;
		return false;
	}

	// If 'base' set, set base and clear queue
	if(req->base || req->effect=="reset") {
		// If base is set, base is set to the requested effect (even if empty)
		if(req->base){this->base_effect = std::make_shared<Effect>(req);}
		// If base is not set and the effect is 'reset', then base is set to default base.
		else if(req->effect == "reset"){this->base_effect = std::make_shared<Effect>(this->default_base_effect);}
		
		this->reset_queue();
		res->message = "Queue emptied";
		return true;
	}


	// Check valid effect first
	bool found = (std::find(VALID_EFFECTS.begin(), VALID_EFFECTS.end(), req->effect) != VALID_EFFECTS.end());
	if(!found) {
		res->message = "Unknown effect: " + req->effect + ". Available effects are fill, fade, wipe, blink, blink_fast, flash, rainbow, rainbow_fill.";
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		return false;
	}

	// Special Case for flash
	if(req->effect == "flash") {
		auto _flash_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(this->flash_delay));
		for(int i = 0; i < this->flash_number; i++){
			this->fill(req->r, req->g, req->b, req->brightness);
			rclcpp::sleep_for(_flash_delay);
			this->fill(0, 0, 0, 0);
			rclcpp::sleep_for(_flash_delay);
		}
		return true;
	}

	// Overwrite the effect at priority or add to priority list. 
	rclcpp::Time curr_time;
	std::shared_ptr<Effect> new_effect = std::make_shared<Effect>(req, this->now());
	this->set_effect_queue(req->priority, new_effect);
	
	return true;
}


bool DexiLEDController::startEffect(std::shared_ptr<Effect> effect){

	auto req = effect->get_effect();

	///////////////
	// Process effect
	if (req->effect != "flash" && req->effect != "fill" && this->current_effect->effect == req->effect &&
	    this->current_effect->r == req->r && this->current_effect->g == req->g && this->current_effect->b == req->b) {
		return true;
	}

	if (req->effect == "fill") {
		this->fill(req->r, req->g, req->b, req->brightness);

	} else if (req->effect == "blink") {
		this->restartTimer(1.0/this->blink_rate);

	} else if (req->effect == "blink_fast") {
		this->restartTimer(1.0/this->blink_fast_rate);

	} else if (req->effect == "fade") {
		this->restartTimer(0.05);

	} else if (req->effect == "wipe") {
		this->restartTimer((double) this->wipe_period/(double) this->led_count);

	}  else if (req->effect == "rainbow_fill") {
		this->restartTimer(this->rainbow_period/255.0);

	} else if (req->effect == "rainbow") {
		this->restartTimer(this->rainbow_period/255.0);

	} else {
		RCLCPP_ERROR(this->get_logger(), "Should never get here");
		return false;
	}
	// finish processing effects request
	///////////////

	// set current effect
	this->curr_effect = effect;
	this->counter = 0;
	this->start_state = this->state;
	this->start_time = this->now();

	return true;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto offboard = std::make_shared<DexiLEDController>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(offboard);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}