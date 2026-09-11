    static int64_t stamp_ns(const rinbo_msgs::msg::Header& h) {
        return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
    }
    std::string topic_age(const std::string& name) const {
        const auto it=seen_.find(name);
        return it==seen_.end()?"":num(recorder::monotonic()-it->second);
    }
    std::string status_json(const std::string& action, const std::string& request_id,
                            const std::string& outcome, int code=0, const std::string& reason="") {
        using recorder::quote;
        std::ostringstream s;
        s<<"{\"protocol\":1,\"schema_version\":2,\"version\":"<<quote(recorder::version)
         <<",\"request_id\":"<<quote(request_id)<<",\"action\":"<<quote(action)
         <<",\"status\":"<<quote(outcome)<<",\"exit_code\":"<<code
         <<",\"reason\":"<<quote(reason)<<",\"recording\":"<<(recording_?"true":"false")
         <<",\"name\":"<<quote(actual_name_)<<",\"run_dir\":"<<quote(active_run_dir_.string())
         <<",\"disk_error\":"<<(disk_error_.empty()?"null":quote(disk_error_))
         <<",\"files_closed\":"<<(!summary_file_.is_open()?"true":"false")
         <<",\"command_source\":"<<quote(command_source_)<<",\"rows\":{\"summary\":"<<summary_flushed_rows_
         <<",\"events\":"<<event_rows_<<",\"commands\":"<<command_rows_
         <<",\"power_samples\":"<<power_rows_<<",\"controller_samples\":"<<controller_rows_<<"},\"topics\":{";
        bool first=true;
        for(const auto* key:{"motor","power","requested","forwarded","controller","safety","safety_detail"}) {
            if(!first)s<<',';
            first=false;
            auto it=seen_.find(key);s<<quote(key)<<":";
            if(it==seen_.end()) s<<"{\"state\":\"unavailable\",\"age_s\":null}";
            else {const auto age=recorder::monotonic()-it->second;
                const auto stamp=source_stamp_.find(key);
                const double source_age=stamp==source_stamp_.end()?-1:(this->now().nanoseconds()-stamp->second)/1e9;
                s<<"{\"state\":"<<quote(age>1.0 || source_age>1.0?"stale":"received")<<",\"age_s\":"<<recorder::number(age)
                 <<",\"last_received_steady_s\":"<<recorder::number(it->second)
                 <<",\"source_stamp_ns\":"<<(stamp==source_stamp_.end()?"null":std::to_string(stamp->second))
                 <<",\"source_age_s\":"<<(stamp==source_stamp_.end()?"null":recorder::number(source_age))
                 <<",\"source_seq\":"<<(source_seq_.count(key)?std::to_string(source_seq_.at(key)):"null")<<"}";}
        }
        s<<"},\"files\":[";first=true;
        if(!active_run_dir_.empty()) for(const auto* file:{"metadata.yaml","summary.csv","events.csv","commands.csv","power_samples.csv","controller_samples.csv"}) {
            const auto path=active_run_dir_/file;
            std::error_code ec;const auto size=std::filesystem::file_size(path,ec);
            if(!first)s<<',';
            first=false;
            s<<"{\"name\":"<<quote(file)<<",\"path\":"<<quote(path.string())<<",\"bytes\":";
            if(ec)s<<"null";else s<<size;s<<"}";
        }
        return s.str()+"]}";
    }
    void control_cb(const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
                    std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
        std::string action,id;
        try {
            std::map<std::string,std::string> values;
            for(const auto& p:request->parameters) {
                if(p.value.type!=4 || values.count(p.name) ||
                   (p.name!="action" && p.name!="name" && p.name!="request_id"))
                    throw std::runtime_error("invalid control parameter");
                values[p.name]=p.value.string_value;
            }
            action=values["action"];id=values["request_id"];
            if(id.empty() || id.size()>80) throw std::runtime_error("request_id required, maximum 80 characters");
            // Serialized service request contains both action and name. Legacy
            // filename messages cannot interleave with this callback.
            bool was_recording=recording_;
            if(action=="start") {
                if(values["name"].empty() || values["name"].size()>128) throw std::runtime_error("name required, maximum 128 characters");
                start_recording(values["name"]);
            } else if(action=="stop") stop_recording();
            else if(action=="status" || action=="export") {if(recording_)flush_files();}
            else throw std::runtime_error("unsupported action");
            const int code=disk_error_.empty()?0:50;
            const auto outcome=code?"disk_error":(action=="start"?(was_recording?"already_recording":"recording"):
                action=="stop"?"stopped":action=="export"?"export_ready":"status");
            response->result.successful=code==0;
            response->result.reason=status_json(action,id,outcome,code,disk_error_);
        } catch(const std::exception& e) {
            response->result.successful=false;
            response->result.reason=status_json(action,id,"rejected",20,e.what());
        }
    }
    void write_raw_headers() {
        std::vector<std::string> command={"receipt_ros_s","message_stamp_ns","message_seq","source"};
        append_motor_cmd_header(command);write_csv_line(commands_file_,command);
        std::vector<std::string> power={"receipt_ros_s","message_stamp_ns","message_seq"};
        append_power_state_header(power);write_csv_line(power_file_,power);
        std::vector<std::string> debug={"receipt_ros_s","message_stamp_ns","message_seq"};
        append_controller_debug_header(debug);write_csv_line(controller_file_,debug);
    }
    void write_command_sample(const rinbo_msgs::msg::MotorCmdStamped& msg, const std::string& source) {
        if(!recording_)return;
        std::vector<std::string> row={num(this->now().seconds()),std::to_string(stamp_ns(msg.header)),std::to_string(msg.header.seq),source};
        const auto saved=motor_cmd_;const bool had=has_motor_cmd_;
        motor_cmd_=msg;has_motor_cmd_=true;append_motor_cmd_row(row);motor_cmd_=saved;has_motor_cmd_=had;
        write_csv_line(commands_file_,row);commands_file_.flush();if(streams_ok())++command_rows_;
    }
    void forwarded_cb(const rinbo_msgs::msg::MotorCmdStamped::SharedPtr msg) {
        forwarded_=*msg;has_forwarded_=true;seen_["forwarded"]=recorder::monotonic();
        source_stamp_["forwarded"]=stamp_ns(msg->header);source_seq_["forwarded"]=msg->header.seq;
        write_command_sample(*msg,"forwarded");
    }
    void write_power_sample(const rinbo_msgs::msg::PowerStateStamped& msg) {
        std::vector<std::string> row={num(this->now().seconds()),std::to_string(stamp_ns(msg.header)),std::to_string(msg.header.seq)};
        append_power_state_row(row);
        const auto now=recorder::monotonic();power_buffer_.push_back({now,row});
        while(!power_buffer_.empty() && (now-power_buffer_.front().first>2.0 || power_buffer_.size()>10000)) power_buffer_.pop_front();
        if(recording_) {write_csv_line(power_file_,row);power_file_.flush();if(streams_ok())++power_rows_;}
    }
    void safety_detail_cb(const std_msgs::msg::String::SharedPtr msg) {
        seen_["safety_detail"]=recorder::monotonic();
        if(!recording_)return;
        // Keep structured native details verbatim even if recorder/producer
        // schemas evolve independently. Never infer a trigger from a 100 Hz row.
        std::vector<std::string> row={num(this->now().seconds()),wall_time_string("%Y-%m-%dT%H:%M:%S%z"),
            "/rinbo/safety_detail","ERROR","native structured safety detail","","","","",""};
        row.insert(row.end(),6,"");
        append(row,{"2","safety_detail","","",msg->data,has_controller_debug_?controller_debug_.controller_state:""});
        write_csv_line(events_file_,row);events_file_.flush();if(streams_ok())++event_rows_;
    }
