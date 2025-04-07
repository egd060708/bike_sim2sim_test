function feedback = bike2dof_lqr_fit_feedback(H_array,K_array,title_name,variant_name)
    feedback = fit_feedback(H_array,K_array,7);
%     disp(feedback)
    disp(title_name);
    f1 = sprintf(strcat(variant_name,'.a = %d;\n'),feedback(1,1));
    f2 = sprintf(strcat(variant_name,'.b = %d;\n'),feedback(1,2));
    f3 = sprintf(strcat(variant_name,'.c = %d;\n'),feedback(1,3));
    f4 = sprintf(strcat(variant_name,'.d = %d;\n'),feedback(1,4));
    f5 = sprintf(strcat(variant_name,'.e = %d;\n'),feedback(1,5));
    f6 = sprintf(strcat(variant_name,'.f = %d;\n'),feedback(1,6));
    f7 = sprintf(strcat(variant_name,'.g = %d;\n'),feedback(1,7));
    f8 = sprintf(strcat(variant_name,'.h = %d;\n'),feedback(1,8));
    fprintf(f1);
    fprintf(f2);
    fprintf(f3);
    fprintf(f4);
    fprintf(f5);
    fprintf(f6);
    fprintf(f7);
    fprintf(f8);
end