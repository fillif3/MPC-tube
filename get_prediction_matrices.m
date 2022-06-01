function [F,H] = get_prediction_matrices(A,B,horizon)
dimenstion_of_state_vector=length(A);
size_of_B=size(B);
dimenstion_of_input_vector=size_of_B(2);
F=zeros(dimenstion_of_state_vector*horizon,dimenstion_of_state_vector);
for i=1:horizon
   F(((i-1)*dimenstion_of_state_vector+1):(i*dimenstion_of_state_vector),:)=A^i;
end
H=zeros(dimenstion_of_state_vector*horizon,horizon*dimenstion_of_input_vector);
for i=1:horizon
   for j=1:min(i,horizon)
        H(((i-1)*dimenstion_of_state_vector+1):(i*dimenstion_of_state_vector),dimenstion_of_input_vector*(j-1)+(1:dimenstion_of_input_vector))=A^(i-j)*B;
   end
end 
end