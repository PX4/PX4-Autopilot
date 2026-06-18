

template <typename TYPE_POLICY, typename TI, bool COPY_FLAG=false>
struct Config{
    struct BASE{
        static constexpr TI NUM_CLASSES = 2<<7;
        static constexpr TI BATCH_SIZE = 32;
        static constexpr TI OUTPUT_DIM = NUM_CLASSES;
        static constexpr TI EMBEDDING_DIM = 32;
        static constexpr TI HIDDEN_DIM = 64;
        static constexpr TI SEQUENCE_LENGTH = 64;
    };
    struct USEFUL: BASE{
        static constexpr TI SEQUENCE_LENGTH = 128;
        static constexpr TI EMBEDDING_DIM = 64;
        static constexpr TI HIDDEN_DIM = 256;
    };
    struct COPY: BASE{
        static constexpr TI NUM_CLASSES = 4;
        static constexpr TI MEM_SIZE = 4;
        static constexpr TI MEM_DELAY = 800;
        static constexpr TI SEQUENCE_LENGTH = MEM_SIZE + MEM_DELAY + MEM_SIZE;
        static constexpr TI EMBEDDING_DIM = 32;
        static constexpr TI HIDDEN_DIM = 32;
    };

//    using PARAMS = BASE;
    using PARAMS = rlt::utils::typing::conditional_t<COPY_FLAG, COPY, USEFUL>;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, PARAMS::SEQUENCE_LENGTH, PARAMS::BATCH_SIZE, 1>;
    using EMBEDDING_LAYER_SPEC = rlt::nn::layers::embedding::Configuration<TYPE_POLICY, TI, PARAMS::NUM_CLASSES, PARAMS::EMBEDDING_DIM>;
    using EMBEDDING_LAYER = rlt::nn::layers::embedding::BindConfiguration<EMBEDDING_LAYER_SPEC>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMS::HIDDEN_DIM, rlt::nn::parameters::groups::Normal, true>;
    using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
//    using GRU2_CONFIG = rlt::nn::layers::gru::Configuration<T, TI, PARAMS::HIDDEN_DIM, rlt::nn::parameters::groups::Normal, true>;
//    using GRU2 = rlt::nn::layers::gru::BindConfiguration<GRU2_CONFIG>;
    using DOWN_PROJECTION_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMS::EMBEDDING_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
    using DOWN_PROJECTION_LAYER_TEMPLATE = rlt::nn::layers::dense::BindConfiguration<DOWN_PROJECTION_LAYER_CONFIG>;
    using DENSE_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMS::OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
    using DENSE_LAYER_TEMPLATE = rlt::nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;

    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

//    using MODULE_CHAIN = Module<EMBEDDING_LAYER, Module<GRU, Module<GRU2, Module<DOWN_PROJECTION_LAYER_TEMPLATE, Module<DENSE_LAYER_TEMPLATE>>>>>;
    using MODULE_CHAIN = Module<EMBEDDING_LAYER, Module<GRU, Module<DOWN_PROJECTION_LAYER_TEMPLATE, Module<DENSE_LAYER_TEMPLATE>>>>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;

    using OUTPUT_TARGET_SHAPE = rlt::tensor::Shape<TI, PARAMS::SEQUENCE_LENGTH, PARAMS::BATCH_SIZE, 1>;
    using T = typename TYPE_POLICY::DEFAULT;
    using OUTPUT_TARGET_SPEC = rlt::tensor::Specification<T, TI, OUTPUT_TARGET_SHAPE>;
    struct ADAM_PARAMS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 0.003;
    };
    using ADAM_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, ADAM_PARAMS>;
    using ADAM = rlt::nn::optimizers::Adam<ADAM_SPEC>;
};
