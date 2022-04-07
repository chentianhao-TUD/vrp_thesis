













def main(number_customer, selected_model_type):
    directory = os.path.join("instances", str(number_customer))
    for entry in os.scandir(directory):
        if entry.path.endswith(".xml") and entry.is_file():
            instance_configuration = entry.path[len(directory) +
                                                1:len(entry.path) - 4]
            print(instance_configuration)

            all_results = list()

            file_name = "_".join(['Results', selected_model_type.name, instance_configuration,
                                  "LastService", "VI1+Tuning_bigM_fix"]) + ".csv"
            result_file = os.path.join('results', str(
                number_customer), selected_model_type.name, file_name)

            with open(result_file, 'w', newline='') as result_csv_file:
                writer = csv.writer(result_csv_file, delimiter=";")
                path = os.path.join(directory, instance_configuration + '.xml')
                instance_amount = read_xml.count_instances(path)


                for instance in range(instance_amount):
                    data = generate_input_data(path, instance)

                    approximated_makespan = 0
                    runtime_approximation = 0


                    data["big_M"] = 10000

                    mdl, variables = build_model(selected_model_type, data)

                    lp_file = "_".join(
                        [selected_model_type.name, instance_configuration, str(instance)]) + ".LP"
                    mdl.write(os.path.join("lp_files", lp_file))

                    for iteration in range(1):
                        log_file = "_".join(
                            [selected_model_type.name, instance_configuration, str(instance), str(iteration)]) + ".LOG"
                        seed = randint(0, 2000000000)

                        status = solve(mdl, log_file, seed)

                        if status == GRB.INFEASIBLE:
                            solve_IIS(mdl)
                        else:
                            tmp_result = [seed, selected_model_type.name, instance_configuration, instance, iteration,
                                          mdl.ObjVal, approximated_makespan, mdl.Runtime, runtime_approximation,
                                          mdl.NodeCount, mdl.IterCount] # Multi Objective
                            all_results.append(tmp_result)

                            writer.writerow(tmp_result)
                            result_csv_file.flush()

                            if mdl.SolCount > 0:
                                sol_file = "_".join([selected_model_type.name, instance_configuration, str(
                                    instance), str(iteration)]) + ".SOL"
                                mdl.write(os.path.join("solutions", sol_file))

